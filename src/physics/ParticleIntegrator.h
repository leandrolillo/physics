/*
 * ParticleIntegrator.h
 *
 *  Created on: Mar 24, 2021
 *      Author: leandro
 */

#include<Math3d.h>
#include<vector>
#include "Particle.h"

class ParticleIntegrator {
public:
  void integrate(real dt, Particle &particle) const {
    particle.acceleration = particle.forceAccumulator * particle.inverseMass;
    particle.setPosition(particle.getPosition() + particle.velocity * dt + 0.5 * particle.acceleration * dt * dt);
    particle.velocity = particle.velocity * powr(particle.damping, dt) + particle.acceleration * dt;
    /**
     * damping [0, 1]: Rough approximation of drag to avoid numerical stability issues - without this objects are likely to accelerate magically.
     * 1 means no drag, 0 means all velocity is lost immediately - something limiting 1 seems reasonable.
     */

  }
};
