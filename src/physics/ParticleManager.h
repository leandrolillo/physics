#pragma once

#include<Math3d.h>
#include<vector>
#include <algorithm>

#include "CollisionDetector.h"
#include "ContactResolver.h"
#include "Force.h"
#include "Particle.h"
#include "ParticleIntegrator.h"

class ParticleManager {
  std::vector<std::unique_ptr<Particle>> particles;
  std::vector<std::unique_ptr<Force>> forces;
  ParticleIntegrator particleIntegrator;
  ContactResolver contactResolver;
  CollisionDetector collisionDetector;
  std::vector<ParticleContact> contacts;

public:
  Particle &addParticle(std::unique_ptr<Particle> particle) {
    if(!particle) {
      throw std::invalid_argument(String(__FILE__) + ":" + std::to_string(__LINE__) + ": Particle can not be null");
    }
    this->particles.push_back(std::move(particle));

    return *this->particles.back();
  }

  const std::vector<std::unique_ptr<Particle>>& getParticles() const {
    return this->particles;
  }

  Force &addForce(std::unique_ptr<Force> force) {
    if(!force) {
      throw std::invalid_argument(String(__FILE__) + ":" + std::to_string(__LINE__) + ": Particle can not be null");
    }

    this->forces.push_back(std::move(force));

    return *this->forces.back();
  }

  Geometry &addScenery(std::unique_ptr<Geometry> scenery) {
    if(!scenery) {
      throw std::invalid_argument(String(__FILE__) + ":" + std::to_string(__LINE__) + ": Particle can not be null");
    }

    this->collisionDetector.addScenery(std::move(scenery));

    return *this->collisionDetector.getScenery().back();
  }

  const std::vector<std::unique_ptr<Geometry>>& getScenery() const {
    return this->collisionDetector.getScenery();
  }

//  void removeParticle(const Particle *particle) {
//    //this->particles.erase(__position)
//    particles.erase(std::remove(particles.begin(), particles.end(), particle), particles.end());
//  }

  void disableParticles() {
    for(auto &particle : this->particles) {
      particle->setStatus(false);
    }
  }

  Particle *nextAvailableParticle() {
    for(auto &particle : this->particles) {
      if(!particle->getStatus()) {
        return particle.get();
      }
    }

    return null;
  }

  CollisionDetector& getCollisionDetector() {
    return this->collisionDetector;
  }

  void clearAccumulators() const {
    for (auto &particle : particles) {
      particle->clearForceAccumulator();
    }
  }

  const std::vector<ParticleContact>& getContacts() const {
    return this->contacts;
  }

  void detectCollisions() {
    contacts = collisionDetector.detectCollisions(this->particles);
  }

  void resolveContacts(real dt) {
    contactResolver.resolve(contacts, dt);
  }

  void step(real dt) {
    applyForces(dt);

    integrate(dt);

    /**
     * generate contacts (collision and contact generators)
     */
    contacts = collisionDetector.detectCollisions(this->particles);
    if(contacts.size() > 0) {
      contactResolver.resolve(contacts, dt);
    }
  }

protected:
  void integrate(real dt) const {
    for (auto &particle : particles) {
      if (particle->getStatus()) {
        particleIntegrator.integrate(dt, *particle.get());
        particle->afterIntegrate(dt);
      }
    }
  }

  void applyForces(real dt) const {
    for (auto &force : forces) {
      force->apply(dt, particles);
    }
  }
};
