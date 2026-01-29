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
