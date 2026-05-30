#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "ParticleIntegrator.h"

namespace {
class TestParticle: public Particle {
public:
  using Particle::Particle;

  const vector &getAccumulatedForce() const {
    return forceAccumulator;
  }
};
}

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

TEST_CASE("Particle validates geometry mass and damping bounds") {
  REQUIRE_THROWS_AS(Particle(std::unique_ptr<Geometry>()), std::invalid_argument);

  Particle particle(std::make_unique<Sphere>(vector(0, 0, 0), 1));
  particle.setMass(0.0f);
  REQUIRE(particle.getMass() == 0.0f);
  REQUIRE(particle.getInverseMass() == 0.0f);

  particle.setInverseMass(0.5f);
  REQUIRE_THAT(particle.getMass(), Catch::Matchers::WithinAbs(2.0f, 0.0001f));
  REQUIRE_THAT(particle.getInverseMass(), Catch::Matchers::WithinAbs(0.5f, 0.0001f));

  Particle noDampingParticle(std::make_unique<Sphere>(vector(0, 0, 0), 1));
  noDampingParticle.setVelocity(vector(4, 0, 0)).setDamping(-1.0f);

  Particle maxDampingParticle(std::make_unique<Sphere>(vector(0, 0, 0), 1));
  maxDampingParticle.setVelocity(vector(4, 0, 0)).setDamping(2.0f);

  ParticleIntegrator integrator;
  integrator.integrate(1.0f, noDampingParticle);
  integrator.integrate(1.0f, maxDampingParticle);

  REQUIRE(noDampingParticle.getVelocity() == vector(0, 0, 0));
  REQUIRE(maxDampingParticle.getVelocity() == vector(4, 0, 0));
}

TEST_CASE("ParticleIntegrator applies accumulated forces and supports clearing them") {
  ParticleIntegrator integrator;
  TestParticle particle(std::make_unique<Sphere>(vector(0, 0, 0), 1));
  particle.setMass(2.0f).setVelocity(vector(2, 0, 0)).setDamping(0.5f);
  particle.applyForce(vector(4, 0, 0));

  REQUIRE(particle.getAccumulatedForce() == vector(4, 0, 0));

  integrator.integrate(1.0f, particle);

  REQUIRE_THAT(particle.getPosition().x, Catch::Matchers::WithinAbs(3.0f, 0.0001f));
  REQUIRE_THAT(particle.getVelocity().x, Catch::Matchers::WithinAbs(3.0f, 0.0001f));
  REQUIRE_THAT(particle.getAcceleration().x, Catch::Matchers::WithinAbs(2.0f, 0.0001f));

  particle.clearForceAccumulator();
  REQUIRE(particle.getAccumulatedForce() == vector(0, 0, 0));

  integrator.integrate(1.0f, particle);

  REQUIRE_THAT(particle.getPosition().x, Catch::Matchers::WithinAbs(6.0f, 0.0001f));
  REQUIRE_THAT(particle.getVelocity().x, Catch::Matchers::WithinAbs(1.5f, 0.0001f));
  REQUIRE_THAT(particle.getAcceleration().x, Catch::Matchers::WithinAbs(0.0f, 0.0001f));
}
