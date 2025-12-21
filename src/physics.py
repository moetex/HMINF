from __future__ import annotations

import math

#p = [b - a for a, b in zip(posA, posB)]

class Physics:
    def calculate(self, sphereA, sphereB):

        vecAB = [sphereB.position.x - sphereA.position.x,sphereB.position.y - sphereA.position.y,sphereB.position.z - sphereA.position.z]

        vecPos
        px = sphereB.position.x - sphereA.position.x
        py = sphereB.position.y - sphereA.position.y
        pz = sphereB.position.z - sphereA.position.z
        teiler = math.sqrt(vecAB[0]^2 + vecAB[1]^2 + vecAB[2]^2)

        vecNormal = [px/teiler, py/teiler, pz/teiler]


        skalarA = vecNormal[0] * sphereA.velocity.x + vecNormal[1] * sphereA.velocity.y + vecNormal[2] * sphereA.velocity.z
        skalarB = vecNormal[0] * sphereB.velocity.x + vecNormal[1] * sphereB.velocity.y + vecNormal[2] * sphereB.velocity.z

        vecA_Para = [vecNormal[0] * skalarA, vecNormal[1] * skalarA, vecNormal[2] * skalarA]
        vecA_Para = [vecNormal[0] * skalarB, vecNormal[1] * skalarB, vecNormal[2] * skalarB]
        result = []
        return result
