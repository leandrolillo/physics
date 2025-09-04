/*
 * CollisionDetector.h
 *
 *  Created on: Apr 5, 2021
 *      Author: leandro
 */

#pragma once
constexpr real sphere_radius=0.1;

#include<vector>
#include"ParticleContact.h"
#include"Particle.h"

#include <Geometry.h>
#include "CollisionTester.h"


class CollisionDetector {
    std::vector <const Geometry *>scenery;
    std::unique_ptr<CollisionTester> intersectionTester;
public:
  CollisionDetector() {
    this->intersectionTester = std::unique_ptr < CollisionTester
        > (new CollisionTester());
  }
  virtual ~CollisionDetector() {
  }

  void setIntersectionTester(CollisionTester *intersectionTester) {
    this->intersectionTester.reset(intersectionTester);
  }

  const CollisionTester* getIntersectionTester() const {
    return this->intersectionTester.get();
  }

  void addScenery(const Geometry *scenery) {
    if (scenery) {
      this->scenery.push_back(scenery);
    }
  }

  const std::vector<const Geometry*>& getScenery() const {
    return this->scenery;
  }

  virtual std::vector<ParticleContact>detectCollisions(const std::vector<Particle *> &particles) const {
        std::vector<ParticleContact> contacts;

        if(particles.size() > 0) {
          /**
           * Iterate without repeating:
           * for i = 0 : n-1
           *  for j = i+1 : n
           *
           *  Note this misses comparing the last element against scenery, thus we have to do it outside of the loop
           */
          for(std::vector<Particle *>::const_iterator iteratorA = particles.begin(); iteratorA != particles.end() - 1; iteratorA++) {
            Particle *particleA = *iteratorA;
            if(particleA && particleA->getStatus() && particleA->getBoundingVolume()) {
              for(auto sceneryIterator : this->scenery)  {
                if(sceneryIterator) {
                  std::vector<GeometryContact> pairContacts = intersectionTester->detectCollision(*(particleA->getBoundingVolume()), *sceneryIterator);
                  if(!pairContacts.empty()) {
                    std::transform(pairContacts.begin(), pairContacts.end(), std::back_inserter(contacts),
                            [&particleA](GeometryContact pairContact) -> ParticleContact {
                            return ParticleContact(particleA,
                                    null,
                                    pairContact.getIntersection(),
                                    pairContact.getNormal(),
                                    pairContact.getRestitution(),
                                    pairContact.getPenetration());
                    });

                    particleA->onCollision(contacts.back());
                  }
                }
              }

              for(std::vector<Particle *>::const_iterator iteratorB = iteratorA+1; iteratorB != particles.end(); iteratorB++) {
                Particle *particleB = *iteratorB;
                if(particleB && particleB->getStatus() && particleB->getBoundingVolume()) {
                  std::vector<GeometryContact> pairContacts = intersectionTester->detectCollision(*particleA->getBoundingVolume(), *particleB->getBoundingVolume());
                  if(!pairContacts.empty()) {
                    std::transform(pairContacts.begin(), pairContacts.end(), std::back_inserter(contacts),
                            [&particleA, &particleB](GeometryContact pairContact) -> ParticleContact {
                            return ParticleContact(particleA,
                                    particleB,
                                    pairContact.getIntersection(),
                                    pairContact.getNormal(),
                                    pairContact.getRestitution(),
                                    pairContact.getPenetration());
                    });

                    particleA->onCollision(contacts.back());
                    particleB->onCollision(contacts.back());
                  }
                }
              }
            }
          }


          /**
           * Check last particle vs scenery since this one was skipped from the loops before
           */
          Particle *particleA = particles.back();
          if(particleA && particleA->getStatus() && particleA->getBoundingVolume()) {
            for(auto sceneryIterator : this->scenery)  {
              if(sceneryIterator) {
                std::vector<GeometryContact> pairContacts = intersectionTester->detectCollision(*(particleA->getBoundingVolume()), *sceneryIterator);
                if(!pairContacts.empty()) {
                  std::transform(pairContacts.begin(), pairContacts.end(), std::back_inserter(contacts),
                          [&particleA](GeometryContact pairContact) -> ParticleContact {
                          return ParticleContact(particleA,
                                  null,
                                  pairContact.getIntersection(),
                                  pairContact.getNormal(),
                                  pairContact.getRestitution(),
                                  pairContact.getPenetration());
                  });

                  particleA->onCollision(contacts.back());
                }
              }
            }
          }
        }

        return contacts;

    }
};
