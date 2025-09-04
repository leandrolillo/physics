/*
 * Gravity.h
 *
 *  Created on: Mar 25, 2021
 *      Author: leandro
 */

#pragma once
#include "Force.h"

class Gravity: public Force {
  vector acceleration;

public:
  Gravity(vector acceleration) {
    this->acceleration = acceleration;
  }

  void apply(real dt, const std::vector<std::unique_ptr<Particle>> &particles) const override {
    for (auto &particle : particles) {
      if (particle->getStatus()) {
        vector force = this->acceleration * particle->getMass();
        particle->applyForce(force);
      }
    }
  }
};
