#include <aikido/constraint/uniform/RnConstantSampler.hpp>

#include <stdexcept>

#include <dart/dart.hpp>

namespace aikido {
namespace constraint {

namespace {

//=============================================================================
template <int N>
class RnConstantSamplerSampleGenerator : public constraint::SampleGenerator
{
public:
  using VectorNd = Eigen::Matrix<double, N, 1>;

  RnConstantSamplerSampleGenerator(
    std::shared_ptr<statespace::Rn<N>> _space, const VectorNd& _value);

  statespace::StateSpacePtr getStateSpace() const override;

  bool sample(statespace::StateSpace::State* _state) override;

  int getNumSamples() const override;

  bool canSample() const override;

private:
  std::shared_ptr<statespace::Rn<N>> mSpace;
  VectorNd mValue;
};

//=============================================================================
template <int N>
RnConstantSamplerSampleGenerator<N>::RnConstantSamplerSampleGenerator(
      std::shared_ptr<statespace::Rn<N>> _space,
      const VectorNd& _value)
  : mSpace(std::move(_space))
  , mValue(_value)
{
  // Do nothing
}

//=============================================================================
template <int N>
statespace::StateSpacePtr
RnConstantSamplerSampleGenerator<N>::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
template <int N>
bool RnConstantSamplerSampleGenerator<N>::sample(
    statespace::StateSpace::State* _state)
{
  mSpace->setValue(static_cast<typename statespace::Rn<N>::State*>(_state), mValue);

  return true;
}

//=============================================================================
template <int N>
int RnConstantSamplerSampleGenerator<N>::getNumSamples() const
{
  return NO_LIMIT;
}

//=============================================================================
template <int N>
bool RnConstantSamplerSampleGenerator<N>::canSample() const
{
  return true;
}

} // namespace anonymous

//=============================================================================
template <int N>
RnConstantSampler<N>::RnConstantSampler(
    std::shared_ptr<statespace::Rn<N>> _space, const VectorNd& _value)
  : mSpace(std::move(_space))
  , mValue(_value)
{
  if (!mSpace)
    throw std::invalid_argument("StateSpace is null.");
}

//=============================================================================
template <int N>
statespace::StateSpacePtr RnConstantSampler<N>::getStateSpace() const
{
  return mSpace;
}

//=============================================================================
template <int N>
std::unique_ptr<constraint::SampleGenerator>
  RnConstantSampler<N>::createSampleGenerator() const
{
  return dart::common::make_unique<RnConstantSamplerSampleGenerator<N>>(
      mSpace, mValue);
}

//=============================================================================
template <int N>
const typename RnConstantSampler<N>::VectorNd&
RnConstantSampler<N>::getConstantValue() const
{
  return mValue;
}

} // namespace constraint
} // namespace aikido
