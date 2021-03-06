#include "ExplorationPlanner/planner/SMCPlanner.h"
#include "ExplorationPlanner/trajectory/TrajectoryValueHandler.h"
#include <omp.h>

bool SMCPlanner::plan(const ExplorationTaskState& state)//检测迭代过程状态，有一次出错，则返回false
{
    for(unsigned int iteration = 0; iteration < parameters_->getNumberOfIterations(); ++iteration)
    {
        if (!iterate(state))
            return false;
    }
    return true;
}

void SMCPlanner::getPlan(std::vector<PlanarPose>& plan) const
{
    Particle pmax = particleset_.getMaxWeightParticle();
    plan = pmax.getTrajectory();
}

bool SMCPlanner::iterate(const ExplorationTaskState& state)
{
    if ( iteration_ > parameters_->getNumberOfIterations() )
        return false;

    if ( iteration_ == 0 )
    {
        // init particleset given the state
        const Particle p(1.0,
                         std::vector<PlanarPose>(getHorizon()+1, state.getRobotPose()),//return Horizon, return robot_pose_
                         std::vector<PlanarRobotVelCmd>(getHorizon(), PlanarRobotVelCmd())//PlanarRobotVelCmd(double vel_lin, double vel_ang, double final_rot, double duration): linear_vel_mps_(vel_lin), angular_vel_rps_(vel_ang),final_rotation_rads(final_rot), duration_secs_(duration)
                         );
        particleset_ = ParticleSet(parameters_->getNumberOfParticles(), p);//迭代集上的路径数，常对象p，函数执行normalizeWeights()
    }

    if (!updateParticles())
        return false;

    evaluateParticles(state);

    ++iteration_;

    return true;
}

bool SMCPlanner::updateParticles()
{
    for (unsigned int i = 0; i < particleset_.getNumberOfParticles(); ++i)
    {
        if ( !generator_->generate(particleset_[i].getTrajectory(),
                                  particleset_[i].getCommands(),
                                  parameters_->getKernel(iteration_),
                                  rng_))
           std::cout << "Warning, trajectory for particle " << i << " had problems. This should not be critical however...\n";
    }
    return true;
}

void SMCPlanner::evaluateParticles(const ExplorationTaskState& state)
{
    const unsigned int num_replications = parameters_->getNumberOfReplicates(iteration_);

    std::vector<TrajectoryValueHandler> rewhandlers(particleset_.getNumberOfParticles(), TrajectoryValueHandler(getDiscount()));
#pragma omp parallel for shared(rewhandlers)
    for (unsigned int i = 0; i < particleset_.getNumberOfParticles(); ++i)
        for(unsigned int replicate = 0; replicate < num_replications; ++replicate)
        {
                rewhandlers[i].addValue( evaluator_->evaluateTrajectory(particleset_[i].getTrajectory(), state.getMap(), rng_) );
        }

    // assign to latest rewards
    latest_rewards_ = rewhandlers;

    // weight the particles
    particleset_.weightParticles( latest_rewards_ );

    // resampling
    const double Neff = particleset_.getEffectiveNumberOfParticles();
    const double Neff_thresh = parameters_->getResamplingThreshold() * particleset_.getNumberOfParticles();
    if (Neff < Neff_thresh)
        particleset_.resample(rng_);

}

