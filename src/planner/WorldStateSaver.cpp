#include "aikido/planner/WorldStateSaver.hpp"

namespace aikido {
namespace planner {

WorldStateSaver::WorldStateSaver(World* world, Options options)
  : mWorld{std::move(world)}, mOptions{options}
{
  if (!mWorld)
    throw std::invalid_argument("World must not be nullptr.");

  if ((mOptions & Options::CONFIGURATIONS) != Options::NONE)
    mWorldState = mWorld->getState();
}

WorldStateSaver::~WorldStateSaver()
{
  if ((mOptions & Options::CONFIGURATIONS) != Options::NONE)
    mWorld->setState(mWorldState);
}

} // namespace planner
} // namespace aikido
