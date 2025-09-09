#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "ParticleIntegrator.h"

TEST_CASE("ParticleIntegrator Test") {
  ParticleIntegrator integrator;
  real dt = 1;
  Particle particle(std::make_unique<Sphere>(vector(0, 0, 0), 1));
  particle.setVelocity(vector(1, 1, 1));
  particle.setDamping(1.0);

  integrator.integrate(dt, particle);
  CHECK(particle.getPosition().x == 1);
  CHECK(particle.getPosition().y == 1);
  CHECK(particle.getPosition().z == 1);

  CHECK(particle.getVelocity().x == 1);
  CHECK(particle.getVelocity().y == 1);
  CHECK(particle.getVelocity().z == 1);

  CHECK(particle.getAcceleration().x == 0);
  CHECK(particle.getAcceleration().y == 0);
  CHECK(particle.getAcceleration().z == 0);

  integrator.integrate(dt, particle);
  CHECK(particle.getPosition().x == 2);
  CHECK(particle.getPosition().y == 2);
  CHECK(particle.getPosition().z == 2);

  CHECK(particle.getVelocity().x == 1);
  CHECK(particle.getVelocity().y == 1);
  CHECK(particle.getVelocity().z == 1);

  CHECK(particle.getAcceleration().x == 0);
  CHECK(particle.getAcceleration().y == 0);
  CHECK(particle.getAcceleration().z == 0);
}
