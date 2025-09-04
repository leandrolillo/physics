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

TEST_CASE("ContactResolver Sphere Sphere collision") {
    CollisionDetector collisionDetector;
    ContactResolver contactResolver;

    real radius = 2.0f;
    std::vector<std::unique_ptr<Particle>> particles;

    particles.push_back(std::make_unique<Particle>(std::make_unique<Sphere>(vector(-1, 0, 0), radius)));
    particles.back()->setVelocity(vector(1, 1, 1));
    particles.back()->setMass(1.0f);
    Particle *sphereParticle = particles.back().get();

    particles.push_back(std::make_unique<Particle>(std::make_unique<Sphere>(vector(2, 1, 1), radius)));
    particles.back()->setVelocity(vector(-1, -1, -1));
    particles.back()->setMass(1.0f);
    Particle *anotherSphereParticle = particles.back().get();

    std::vector<ParticleContact> contacts = collisionDetector.detectCollisions(particles);
    contactResolver.resolve(contacts, 0.1f);

    vector delta = anotherSphereParticle->getPosition() - sphereParticle->getPosition();
    vector normal = delta.normalizado();
    real actualPenetration = (radius + radius) - delta.modulo();
    real deltaVelocity = (sphereParticle->getVelocity() - anotherSphereParticle->getVelocity()) * normal;

    REQUIRE_THAT(actualPenetration, Catch::Matchers::WithinAbs(0, 0.0001f));
    REQUIRE_THAT(deltaVelocity, Catch::Matchers::WithinAbs(-2.41209f, 0.0001f));
}

TEST_CASE("ContactResolver Sphere Plane collision") {
    CollisionDetector collisionDetector;
    ContactResolver contactResolver;

    real radius = 2.0f;
    std::vector<std::unique_ptr<Particle>> particles;

    particles.push_back(std::make_unique<Particle>(std::make_unique<Sphere>(vector(0, 0, 0), radius)));
    Particle *sphereParticle = particles.back().get();
    particles.back()->setVelocity(vector(0, -1, 0));
    particles.back()->setMass(1.0f);

    particles.push_back(std::make_unique<Particle>(std::make_unique<Plane>(vector(0, 0, 0), vector(0, 1, 0))));
    Particle *planeParticle = particles.back().get();

    std::vector<ParticleContact> contacts = collisionDetector.detectCollisions(particles);
    contactResolver.resolve(contacts, 0.1f);

    vector delta =  sphereParticle->getPosition() - planeParticle->getPosition();
    vector normal = -((Plane &)planeParticle->getBoundingVolume()).getNormal();
    real actualPenetration = delta * normal;
    real deltaVelocity = (sphereParticle->getVelocity() - planeParticle->getVelocity()) * normal;

    REQUIRE(0.0f >= actualPenetration);
    REQUIRE(0.0f >= deltaVelocity);
}

TEST_CASE("Forces") {
  Gravity gravity(vector(0, -9.8, 0));
}

//void testPlaneSphere(PlaygroundRunner *runner) {
//
//}
//
//void testSphereAabb(PlaygroundRunner *runner) {
//
//}
//
//void testAabbSphere(PlaygroundRunner *runner) {
//
//}
