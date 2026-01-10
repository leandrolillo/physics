#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "ParticleManager.h"
#include "Gravity.h"

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
