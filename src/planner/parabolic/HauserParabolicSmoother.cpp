#include <chrono>
#include <cmath>
#include <aikido/util/VanDerCorput.hpp>
#include "HauserMath.h"
#include "HauserParabolicSmoother.hpp"
#include "ParabolicUtil.hpp"
#include "Config.h"

using namespace ParabolicRamp;

namespace aikido {
namespace planner {
namespace parabolic {

class RampRandomGenerator : public RandomNumberGeneratorBase
{
public:
    RampRandomGenerator(aikido::util::RNG::result_type _rngSeed)
        : rng_(_rngSeed), distribution_(0.0, 1.0)
    {
    }

    virtual Real Rand() { return distribution_(rng_); }
    aikido::util::RNGWrapper<std::mt19937> rng_;
    std::uniform_real_distribution<>       distribution_;
};


class SmootherFeasibilityCheckerBase : public ParabolicRamp::FeasibilityCheckerBase
{
public:
    SmootherFeasibilityCheckerBase(aikido::constraint::TestablePtr testable,
                                   double checkResolution = 0.2)
        : testable_(std::move(testable)),
          checkResolution_(checkResolution)
    {
      interpolator_ = std::make_shared<aikido::statespace::GeodesicInterpolator>
              (testable_->getStateSpace());
    }

    virtual bool ConfigFeasible(ParabolicRamp::Vector const &x)
    {
      statespace::StateSpacePtr stateSpace = testable_->getStateSpace();
      Eigen::VectorXd eigX = toEigen(x);
      auto state = stateSpace->createState();
      stateSpace->expMap(eigX, state);
      return testable_->isSatisfied(state);
    }

    virtual bool SegmentFeasible(ParabolicRamp::Vector const &a,
                                 ParabolicRamp::Vector const &b)
    {
       statespace::StateSpacePtr stateSpace = testable_->getStateSpace();
       Eigen::VectorXd eigA = toEigen(a);
       Eigen::VectorXd eigB = toEigen(b);

       auto testState = stateSpace->createState();
       auto startState = stateSpace->createState();
       auto goalState = stateSpace->createState();
       stateSpace->expMap(eigA, startState);
       stateSpace->expMap(eigB, goalState);

       aikido::util::VanDerCorput vdc{1, true, true, checkResolution_};

       for (const auto alpha : vdc)
       {
         interpolator_->interpolate(startState, goalState, alpha, testState);
         if (!testable_->isSatisfied(testState))
         {
           return false;
         }
       }
       return true;
    }

private:
    aikido::constraint::TestablePtr                           testable_;
    double                                                    checkResolution_;
    std::shared_ptr<aikido::statespace::GeodesicInterpolator> interpolator_;
};

HauserParabolicSmoother::HauserParabolicSmoother(aikido::constraint::TestablePtr testable,
                                                 double timelimit,
                                                 double blendRadius,
                                                 int blendIterations,
                                                 double tolerance)
    : blendRadius_(blendRadius),
      blendIterations_(blendIterations),
      timelimit_(timelimit*1e3),
      tolerance_(tolerance)
{
    if(timelimit_ < 0.0)
      throw std::runtime_error("Timelimit should be non-negative");
    if(blendIterations_ <= 0)
      throw std::runtime_error("Blend iterations should be positive");
    if(blendRadius_ <= 0.0)
      throw std::runtime_error("Blend radius should be positive");
    if(tolerance_ < 0.0)
      throw std::runtime_error("Tolerance should be non-negative");

    checkerBase_ = std::make_shared<SmootherFeasibilityCheckerBase>(testable);
    feasibilityChecker_ = std::make_shared<ParabolicRamp::RampFeasibilityChecker>
            (checkerBase_.get(), static_cast<ParabolicRamp::Real>(tolerance_));
}


bool HauserParabolicSmoother::doShortcut(ParabolicRamp::DynamicPath& dynamicPath,
                                         aikido::util::RNG::result_type _rngSeed)
{
    std::chrono::time_point<std::chrono::system_clock> startTime, currentTime;
    startTime = std::chrono::system_clock::now();
    double elapsedTime = 0;

    RampRandomGenerator* rndGnr= new RampRandomGenerator(_rngSeed);

    while (elapsedTime < timelimit_ && dynamicPath.ramps.size() > 3)
    {
        dynamicPath.Shortcut(1, *feasibilityChecker_, rndGnr);

        currentTime = std::chrono::system_clock::now();
        elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
            currentTime - startTime).count();
    }
    return true;
}


bool HauserParabolicSmoother::doBlend(ParabolicRamp::DynamicPath& dynamicPath)
{
    // Mark all of the ramps in the initial trajectory as "original". We'll
    // only try to blend transitions between these ramps.
    for(unsigned int i=0;i<dynamicPath.ramps.size();++i)
    {
        dynamicPath.ramps[i].blendAttempts = 0;
    }

    double dtShortcut = blendRadius_;

    // tryBlend always starts at the beginning of the trajectory.
    // Without this bookkeeping, tryBlend would could any blend that fails multiple times.
    for (int attempt = 0; attempt < blendIterations_; ++attempt)
    {
        while (tryBlend(dynamicPath, attempt, dtShortcut));

        dtShortcut /= 2.;
    }
    return true;
}

bool HauserParabolicSmoother::tryBlend(ParabolicRamp::DynamicPath& dynamicPath,
                                       int attempt, double dtShortcut)
{
    // blending can completely remove waypoints from the trajectory in the case
    // that two waypoints are closer than _blendRadius together - which means
    // that waypoint indicies can change between iterations of the algorithm.

    size_t const numRamps = dynamicPath.ramps.size();
    double const tMax = dynamicPath.GetTotalTime();
    double t = 0;

    for (size_t iRamp = 0; iRamp < numRamps-1; ++iRamp)
    {
        ParabolicRamp::ParabolicRampND &rampNd = dynamicPath.ramps[iRamp];
        t += rampNd.endTime;

        if (needsBlend(rampNd) && rampNd.blendAttempts == attempt)
        {
            double const t1 = std::max(t - dtShortcut, - ParabolicRamp::EpsilonT);
            double const t2 = std::min(t + dtShortcut, tMax + ParabolicRamp::EpsilonT);

            bool const success = dynamicPath.TryShortcut(t1, t2, *feasibilityChecker_.get());

            if (success)
            {
                return true;
            } else {
                rampNd.blendAttempts++;
            }
        }
    }

    return false;
}

bool HauserParabolicSmoother::needsBlend(ParabolicRamp::ParabolicRampND const &rampNd)
{
    for (size_t idof = 0; idof < rampNd.dx1.size(); ++idof)
    {
        if (std::fabs(rampNd.dx1[idof]) > ParabolicRamp::EpsilonV)
        {
            return false;
        }
    }
    return true;
}

} // namespace parabolic
} // namespace planner
} // namespace aikido
