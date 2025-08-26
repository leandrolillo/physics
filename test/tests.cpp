#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "ParticleManager.h"

TEST_CASE("CollisionDetector Sphere Sphere collision") {
    CollisionDetector collisionDetector;

    real radius = 2.0f;
    std::vector<Particle*> particles;

    Particle sphereParticle(new Sphere(vector(-1, 0, 0), radius));
    sphereParticle.setMass(1.0f);

    Particle anotherSphereParticle(new Sphere(vector(2, 1, 1), radius));
    anotherSphereParticle.setMass(1.0f);

    particles.push_back(&sphereParticle);
    particles.push_back(&anotherSphereParticle);

    std::vector<ParticleContact> contacts = collisionDetector.detectCollisions(particles);
    ParticleContact contact = *contacts.begin();

    REQUIRE((void*) &sphereParticle == (void*) contact.getParticleA());
    REQUIRE((void*) &anotherSphereParticle == (void*) contact.getParticleB());

    vector expectedNormal = (sphereParticle.getPosition() - anotherSphereParticle.getPosition()).normalizado();
    real expectedPenetration = (radius + radius) - (sphereParticle.getPosition() - anotherSphereParticle.getPosition()).modulo();

    REQUIRE(expectedNormal == contact.getNormal());
    REQUIRE(expectedPenetration == contact.getPenetration());
}

TEST_CASE("ContactResolver Sphere Sphere collision") {
    CollisionDetector collisionDetector;
    ContactResolver contactResolver;

    real radius = 2.0f;
    std::vector<Particle*> particles;

    Particle sphereParticle(new Sphere(vector(-1, 0, 0), radius));
    sphereParticle.setVelocity(vector(1, 1, 1));
    sphereParticle.setMass(1.0f);

    Particle anotherSphereParticle(new Sphere(vector(2, 1, 1), radius));
    anotherSphereParticle.setVelocity(vector(-1, -1, -1));
    anotherSphereParticle.setMass(1.0f);

    particles.push_back(&sphereParticle);
    particles.push_back(&anotherSphereParticle);

    std::vector<ParticleContact> contacts = collisionDetector.detectCollisions(particles);
    contactResolver.resolve(contacts, 0.1f);

    vector delta = anotherSphereParticle.getPosition() - sphereParticle.getPosition();
    vector normal = delta.normalizado();
    real actualPenetration = (radius + radius) - delta.modulo();
    real deltaVelocity = (sphereParticle.getVelocity() - anotherSphereParticle.getVelocity()) * normal;

    REQUIRE_THAT(actualPenetration, Catch::Matchers::WithinAbs(0, 0.0001f));
    REQUIRE_THAT(deltaVelocity, Catch::Matchers::WithinAbs(0, 0.0001f));
}

TEST_CASE("ContactResolver Sphere Plane collision") {
    CollisionDetector collisionDetector;
    ContactResolver contactResolver;

    real radius = 2.0f;
    std::vector<Particle*> particles;

    Particle sphereParticle(new Sphere(vector(0, 0, 0), radius));
    sphereParticle.setVelocity(vector(0, -1, 0));
    sphereParticle.setMass(1.0f);

    Particle planeParticle(new Plane(vector(0, 0, 0), vector(0, 1, 0)));

    particles.push_back(&sphereParticle);
    particles.push_back(&planeParticle);

    std::vector<ParticleContact> contacts = collisionDetector.detectCollisions(particles);
    contactResolver.resolve(contacts, 0.1f);

    vector delta =  sphereParticle.getPosition() - planeParticle.getPosition();
    vector normal = -((Plane *)planeParticle.getBoundingVolume())->getNormal();
    real actualPenetration = delta * normal;
    real deltaVelocity = (sphereParticle.getVelocity() - planeParticle.getVelocity()) * normal;

    REQUIRE(0.0f >= actualPenetration);
    REQUIRE(0.0f >= deltaVelocity);
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
