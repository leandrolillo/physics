#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "ParticleManager.h"
#include "Gravity.h"

TEST_CASE("CollisionDetector Sphere Sphere collision") {
    CollisionDetector collisionDetector;

    real radius = 2.0f;
    std::vector<std::unique_ptr<Particle>> particles;

    particles.push_back(std::make_unique<Particle>(std::make_unique<Sphere>(vector(-1, 0, 0), radius)));
    particles.back()->setMass(1.0f);
    Particle * sphereParticle = particles.back().get();

    particles.push_back(std::make_unique<Particle>(std::make_unique<Sphere>(vector(2, 1, 1), radius)));
    particles.back()->setMass(1.0f);
    Particle * anotherSphereParticle = particles.back().get();

    std::vector<ParticleContact> contacts = collisionDetector.detectCollisions(particles);
    REQUIRE(contacts.size() > 0);
    ParticleContact contact = *contacts.begin();

    REQUIRE(sphereParticle == contact.getParticleA());
    REQUIRE(anotherSphereParticle == contact.getParticleB());

    vector expectedNormal = (sphereParticle->getPosition() - anotherSphereParticle->getPosition()).normalizado();
    real expectedPenetration = (radius + radius) - (sphereParticle->getPosition() - anotherSphereParticle->getPosition()).modulo();

    REQUIRE(expectedNormal == contact.getNormal());
    REQUIRE(expectedPenetration == contact.getPenetration());
}

TEST_CASE("CollisionDetector addScenery removeScenery") {
  CollisionDetector collisionDetector;

  Sphere &sphere = (Sphere &)collisionDetector.addScenery(std::make_unique<Sphere>(vector(2, 1, 1), 2.0));
  CHECK(collisionDetector.getScenery().size() == 1);

  collisionDetector.removeScenery(sphere);
  CHECK(collisionDetector.getScenery().size() == 0);
}

TEST_CASE("CollisionDetector skips inactive and separated particles") {
  CollisionDetector collisionDetector;

  std::vector<std::unique_ptr<Particle>> particles;
  particles.push_back(std::make_unique<Particle>(std::make_unique<Sphere>(vector(0, 0, 0), 1.0)));
  particles.push_back(std::make_unique<Particle>(std::make_unique<Sphere>(vector(10, 0, 0), 1.0)));

  REQUIRE(collisionDetector.detectCollisions(particles).empty());

  particles.back()->setPosition(vector(0.5f, 0, 0)).setStatus(false);
  REQUIRE(collisionDetector.detectCollisions(particles).empty());
}

TEST_CASE("CollisionDetector detects single particle against scenery and keeps restitution") {
  CollisionDetector collisionDetector;
  collisionDetector.setRestitution(0.4f);

  std::vector<std::unique_ptr<Particle>> particles;
  particles.push_back(std::make_unique<Particle>(std::make_unique<Sphere>(vector(0, 0, 0), 2.0)));
  Particle *particle = particles.back().get();

  collisionDetector.addScenery(std::make_unique<Sphere>(vector(1, 0, 0), 2.0));
  std::vector<ParticleContact> contacts = collisionDetector.detectCollisions(particles);

  REQUIRE(contacts.size() == 1);
  REQUIRE(contacts.front().getParticleA() == particle);
  REQUIRE(contacts.front().getParticleB() == null);
  REQUIRE_THAT(contacts.front().getRestitution(), Catch::Matchers::WithinAbs(0.4f, 0.0001f));
  REQUIRE(contacts.front().getPenetration() > 0.0f);
}
