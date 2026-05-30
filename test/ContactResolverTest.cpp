#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "ParticleManager.h"
#include "Gravity.h"

namespace {
class TrackingParticle: public Particle {
public:
  using Particle::Particle;

  int resolvedCollisions = 0;

  void onCollisionResolved(const ParticleContact &contact) override {
    Particle::onCollisionResolved(contact);
    resolvedCollisions++;
  }
};
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
    REQUIRE_THAT(deltaVelocity, Catch::Matchers::WithinAbs(-2.8643f, 0.0001f));
}

TEST_CASE("ContactResolver leaves separating particles unchanged") {
    ContactResolver contactResolver;

    Particle particleA(std::make_unique<Sphere>(vector(0, 0, 0), 1.0));
    particleA.setVelocity(vector(1, 0, 0)).setMass(1.0f);

    Particle particleB(std::make_unique<Sphere>(vector(3, 0, 0), 1.0));
    particleB.setVelocity(vector(0, 0, 0)).setMass(1.0f);

    ParticleContact contact(&particleA, &particleB, vector(1.5f, 0, 0), vector(1, 0, 0), 0.8f);
    contactResolver.resolveVelocity(contact, 1.0f);

    REQUIRE(particleA.getVelocity() == vector(1, 0, 0));
    REQUIRE(particleB.getVelocity() == vector(0, 0, 0));
}

TEST_CASE("ContactResolver resolves particle against immovable scenery and invokes callbacks") {
    ContactResolver contactResolver;

    TrackingParticle particle(std::make_unique<Sphere>(vector(0, 0, 0), 1.0));
    particle.setVelocity(vector(-2, 0, 0)).setMass(1.0f);

    ParticleContact contact(&particle, null, vector(0, 0, 0), vector(1, 0, 0), 0.5f, 2.0f);
    contactResolver.resolve({contact}, 1.0f);

    REQUIRE_THAT(particle.getVelocity().x, Catch::Matchers::WithinAbs(1.0f, 0.0001f));
    REQUIRE_THAT(particle.getPosition().x, Catch::Matchers::WithinAbs(2.0f, 0.0001f));
    REQUIRE(particle.resolvedCollisions == 1);
}

TEST_CASE("ContactResolver ignores contacts with no movable particles") {
    ContactResolver contactResolver;

    Particle particleA(std::make_unique<Sphere>(vector(0, 0, 0), 1.0));
    particleA.setMass(0.0f).setVelocity(vector(-1, 0, 0));

    Particle particleB(std::make_unique<Sphere>(vector(0.5f, 0, 0), 1.0));
    particleB.setMass(0.0f).setVelocity(vector(1, 0, 0));

    ParticleContact contact(&particleA, &particleB, vector(0.25f, 0, 0), vector(1, 0, 0), 1.0f, 1.5f);
    contactResolver.resolve({contact}, 1.0f);

    REQUIRE(particleA.getVelocity() == vector(-1, 0, 0));
    REQUIRE(particleB.getVelocity() == vector(1, 0, 0));
    REQUIRE(particleA.getPosition() == vector(0, 0, 0));
    REQUIRE(particleB.getPosition() == vector(0.5f, 0, 0));
}
