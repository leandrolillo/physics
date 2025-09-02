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
  std::vector<Particle*> particles;
  std::vector<Force*> forces;
  ParticleIntegrator particleIntegrator;
  ContactResolver contactResolver;
  CollisionDetector collisionDetector;
  std::vector<ParticleContact> contacts;

public:
  void addParticle(Particle *particle) {
    this->particles.push_back(particle);
  }

  const std::vector<Particle*>& getParticles() const {
    return this->particles;
  }

  void addForce(Force *force) {
    this->forces.push_back(force);
  }

  void addScenery(const Geometry *scenery) {
    this->collisionDetector.addScenery(scenery);
  }

  const std::vector<const Geometry*>& getScenery() const {
    return this->collisionDetector.getScenery();
  }

  void removeParticle(const Particle *particle) {
    //this->particles.erase(__position)
    particles.erase(std::remove(particles.begin(), particles.end(), particle), particles.end());
  }

  CollisionDetector& getCollisionDetector() {
    return this->collisionDetector;
  }

  void setIntersectionTester(CollisionTester *intersectionTester) {
    this->collisionDetector.setIntersectionTester(intersectionTester);
  }

  void clearAccumulators() const {
    for (std::vector<Particle*>::const_iterator iterator = particles.begin(); iterator != particles.end(); iterator++) {
      (*iterator)->clearForceAccumulator();
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
    contactResolver.resolve(contacts, dt);

  }

protected:
  void integrate(real dt) const {
    for (std::vector<Particle*>::const_iterator iterator = particles.begin(); iterator != particles.end(); iterator++) {
      Particle *particle = *iterator;
      if (particle != null && particle->getStatus()) {
        particleIntegrator.integrate(dt, *particle);
        particle->afterIntegrate(dt);
      }
    }
  }

  void applyForces(real dt) const {
    for (std::vector<Force*>::const_iterator iterator = forces.begin(); iterator != forces.end(); iterator++) {
      (*iterator)->apply(dt, particles);
    }
  }
};
