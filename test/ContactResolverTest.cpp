#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "ParticleManager.h"
#include "Gravity.h"

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
    REQUIRE_THAT(deltaVelocity, Catch::Matchers::WithinAbs(-2.8643f, 0.0001f));
}
