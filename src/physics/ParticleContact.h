/*
 * Contact.h
 *
 *  Created on: Mar 24, 2021
 *      Author: leandro
 */

#pragma once
#include "BaseContact.h"
#include "Particle.h"

class ParticleContact: public BaseContact {
public:
  static ParticleContact noContact;
  private:
  Particle *particleA;
  Particle *particleB;
  real relativeSpeed;
  real restitution {0}; //how much energy to dissipate - 0 = all 1 = none. No energy dissipation could lead to numerical instability

public:
  ParticleContact(Particle *particleA, Particle *particleB, const vector &intersection, const vector &normal, real restitution,
      real penetration = 0.0) : BaseContact(intersection, normal, penetration) {
    this->particleA = particleA;
    this->particleB = particleB;
    this->restitution = restitution;

    vector relativeVelocity = particleA->getVelocity();
    if (particleB != null) {
      relativeVelocity -= particleB->getVelocity();
    }
    relativeSpeed = relativeVelocity * normal;

  }

  Particle* getParticleA() const {
    return this->particleA;
  }

  Particle* getParticleB() const {
    return this->particleB;
  }

  real getRelativeSpeed() const {
    return this->relativeSpeed;
  }

  real getRestitution() const {
    return this->restitution;
  }
};
