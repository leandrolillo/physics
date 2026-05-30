#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "ParticleManager.h"
#include "Gravity.h"

namespace {
class TrackingParticle: public Particle {
public:
  using Particle::Particle;

  int afterIntegrateCalls = 0;

  const vector &getAccumulatedForce() const {
    return forceAccumulator;
  }

  void afterIntegrate(real dt) override {
    Particle::afterIntegrate(dt);
    afterIntegrateCalls++;
  }
};
}

TEST_CASE("ParticleManager") {
  ParticleManager particleManager;

  particleManager.addParticle(std::make_unique<Particle>(std::make_unique<Sphere>(vector(-1, 0, 0), 2.0)));
  CHECK(particleManager.getParticles().size() == 1);


  Sphere &sphere = (Sphere &)particleManager.addScenery(std::make_unique<Sphere>(vector(2, 1, 1), 2.0));
  CHECK(particleManager.getScenery().size() == 1);

  particleManager.removeScenery(sphere);
  CHECK(particleManager.getScenery().size() == 0);
}

TEST_CASE("Forces") {
  Gravity gravity(vector(0, -9.8, 0));
}

TEST_CASE("ParticleManager validates null inputs and disabled particles") {
  ParticleManager particleManager;

  REQUIRE_THROWS_AS(particleManager.addParticle(std::unique_ptr<Particle>()), std::invalid_argument);
  REQUIRE_THROWS_AS(particleManager.addForce(std::unique_ptr<Force>()), std::invalid_argument);
  REQUIRE_THROWS_AS(particleManager.addScenery(std::unique_ptr<Geometry>()), std::invalid_argument);

  Particle &firstParticle = particleManager.addParticle(std::make_unique<Particle>(std::make_unique<Sphere>(vector(0, 0, 0), 1.0)));
  particleManager.addParticle(std::make_unique<Particle>(std::make_unique<Sphere>(vector(3, 0, 0), 1.0)));

  REQUIRE(particleManager.nextAvailableParticle() == null);

  particleManager.disableParticles();

  REQUIRE(particleManager.nextAvailableParticle() == &firstParticle);
}

TEST_CASE("ParticleManager step applies forces and integrates only active particles") {
  ParticleManager particleManager;
  particleManager.addForce(std::make_unique<Gravity>(vector(0, -9.8f, 0)));

  auto activeParticle = std::make_unique<TrackingParticle>(std::make_unique<Sphere>(vector(0, 0, 0), 1.0));
  activeParticle->setMass(2.0f).setDamping(1.0f);
  TrackingParticle *activeParticlePtr = activeParticle.get();
  particleManager.addParticle(std::move(activeParticle));

  auto inactiveParticle = std::make_unique<TrackingParticle>(std::make_unique<Sphere>(vector(0, 10, 0), 1.0));
  inactiveParticle->setStatus(false);
  TrackingParticle *inactiveParticlePtr = inactiveParticle.get();
  particleManager.addParticle(std::move(inactiveParticle));

  particleManager.step(1.0f);

  REQUIRE_THAT(activeParticlePtr->getPosition().y, Catch::Matchers::WithinAbs(-4.9f, 0.0001f));
  REQUIRE_THAT(activeParticlePtr->getVelocity().y, Catch::Matchers::WithinAbs(-9.8f, 0.0001f));
  REQUIRE(activeParticlePtr->getAcceleration() == vector(0, -9.8f, 0));
  REQUIRE(activeParticlePtr->afterIntegrateCalls == 1);

  REQUIRE(inactiveParticlePtr->getPosition() == vector(0, 10, 0));
  REQUIRE(inactiveParticlePtr->getVelocity() == vector(0, 0, 0));
  REQUIRE(inactiveParticlePtr->afterIntegrateCalls == 0);

  activeParticlePtr->applyForce(vector(3, 2, 1));
  REQUIRE(activeParticlePtr->getAccumulatedForce() == vector(3, 2, 1));
  particleManager.clearAccumulators();
  REQUIRE(activeParticlePtr->getAccumulatedForce() == vector(0, 0, 0));
}
