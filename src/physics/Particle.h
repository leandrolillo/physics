/*
 * Particle.h
 *
 *  Created on: Mar 18, 2021
 *      Author: leandro
 */

#pragma once

#include <Math3d.h>
#include <Geometry.h>

class ParticleContact;

class Particle {
  friend class ParticleIntegrator;

protected:
  std::unique_ptr<Geometry> boundingVolume;
  vector velocity = {0, 0, 0};
  vector acceleration = {0, 0, 0};

  real inverseMass = 0.0f;
  real mass = 0.0f;

  bool _status = true;

  /**
   * damping [0, 1]: Rough approximation of drag to avoid numerical stability issues - without this objects are likely to accelerate magically.
   * 1 means no drag, 0 means all velocity is lost immediately - something limiting 1 seems reasonable.
   */
  real damping;

  vector forceAccumulator {0, 0, 0};

public:
  Particle(std::unique_ptr<Geometry> geometry) {
    if(geometry) {
      this->boundingVolume = std::move(geometry);
    } else {
      throw std::invalid_argument("geometry can not be null");
    }

    /*Set default values that make the particle "active", otherwise it is confussing when nothing happens*/
    setDamping(nextafter(1, 0));
    setMass(1.0);
  }

  virtual ~Particle() {

  }

  virtual const Geometry &getBoundingVolume() const {
    return *this->boundingVolume.get();
  }

  virtual void afterIntegrate(real dt) {
  }

  virtual void onCollision(const ParticleContact &contact) {

  }
  virtual void onCollisionResolved(const ParticleContact &contact) {

  }

  bool getStatus() const {
    return this->_status;
  }

  Particle &setStatus(bool active) {
    this->_status = active;
    return *this;
  }

  Particle &setPosition(const vector &position) {
    this->boundingVolume->setOrigin(position);
    return *this;
  }

  const vector& getPosition() const {
    return this->boundingVolume->getOrigin();
  }

  Particle &setVelocity(const vector &velocity) {
    this->velocity = velocity;
    return *this;
  }

  const vector& getVelocity() const {
    return this->velocity;
  }

  Particle &setAcceleration(const vector &acceleration) {
    this->acceleration = acceleration;
    return *this;
  }

  const vector& getAcceleration() const {
    return this->acceleration;
  }

  Particle &setMass(real mass) {
    if (mass == 0.0f) {
      this->inverseMass = (real) 0.0;
    } else {
      this->inverseMass = (real) 1.0 / mass;
    }

    this->mass = mass;
    return *this;
  }

  Particle &setInverseMass(real inverseMass) {
    if (inverseMass == 0.0f) {
      this->mass = (real) 0.0;
    } else {
      this->mass = (real) 1.0 / inverseMass;
    }

    this->inverseMass = inverseMass;
    return *this;
  }

  const real getMass() const {
    return this->mass;
  }

  const real getInverseMass() const {
    return this->inverseMass;
  }

  Particle &setDamping(real damping) {
    this->damping = std::max((real)0.0, std::min((real)1.0, damping));
    return *this;
  }

  Particle & clearForceAccumulator() {
    this->forceAccumulator = vector(0, 0, 0);
    return *this;
  }

  Particle & applyForce(vector force) {
    this->forceAccumulator += force;
    return *this;
  }
};
