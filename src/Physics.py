from __future__ import annotations

import math

#p = [b - a for a, b in zip(posA, posB)]

class Physics:
    def calculateVelocityAfterCollision_Spheres(self, sphereA, sphereB):

        vecAB = [sphereB.position.x - sphereA.position.x,sphereB.position.y - sphereA.position.y,sphereB.position.z - sphereA.position.z]

        vecA_v = [sphereA.velocity.x,sphereA.velocity.y,sphereA.velocity.z]
        vecB_v = [sphereB.velocity.x, sphereB.velocity.y, sphereB.velocity.z]

        teiler = math.sqrt(vecAB[0]^2 + vecAB[1]^2 + vecAB[2]^2)

        vecNormal = [vecAB[0]/teiler, vecAB[1]/teiler, vecAB[2]/teiler]


        skalarA = vecNormal[0] * vecA_v[0] + vecNormal[1] * vecA_v[1] + vecNormal[2] * vecA_v[2]
        skalarB = vecNormal[0] * sphereB.velocity.x + vecNormal[1] * sphereB.velocity.y + vecNormal[2] * sphereB.velocity.z

        vecA_Para = [vecNormal[0] * skalarA, vecNormal[1] * skalarA, vecNormal[2] * skalarA]
        vecB_Para = [vecNormal[0] * skalarB, vecNormal[1] * skalarB, vecNormal[2] * skalarB]

        vecA_Senk = [vecA_v[0] - vecA_Para[0], vecA_v[1] - vecA_Para[1], vecA_v[2] - vecA_Para[2]]
        vecB_Senk = [vecB_v[0] - vecB_Para[0], vecB_v[1] - vecB_Para[1], vecB_v[2] - vecB_Para[2]]


        a = self.collision_in_direction_n(sphereA.mass, sphereB.mass, vecA_Para, vecB_Para)

        vecA_Para_after = [a[0],a[1],a[2]]
        vecB_Para_after = [a[3],a[4],a[5]]

        vecA_after = [vecA_Para_after[0] + vecA_Senk[0],vecA_Para_after[1] + vecA_Senk[1],vecA_Para_after[2] + vecA_Senk[2]]
        vecB_after = [vecB_Para_after[0] + vecB_Senk[0],vecB_Para_after[1] + vecB_Senk[1],vecB_Para_after[2] + vecB_Senk[2]]
        result = [vecA_after[0],vecA_after[1],vecA_after[2],vecB_after[3],vecB_after[4],vecB_after[5]]
        return result


    def collision_in_direction_n(self, massA, massB, vecA_Para, vecB_Para):

        vecA_Para_after_x = ((massA - massB) * vecA_Para[0] + 2 * massB * vecB_Para[0]) / (massA + massB)
        vecA_Para_after_y = ((massA - massB) * vecA_Para[1] + 2 * massB * vecB_Para[1]) / (massA + massB)
        vecA_Para_after_z = ((massA - massB) * vecA_Para[2] + 2 * massB * vecB_Para[2]) / (massA + massB)

        vecB_Para_after_x = ((massA - massB) * vecB_Para[0] + 2 * massA * vecA_Para[0]) / (massA + massB)
        vecB_Para_after_y = ((massA - massB) * vecB_Para[1] + 2 * massA * vecA_Para[1]) / (massA + massB)
        vecB_Para_after_z = ((massA - massB) * vecB_Para[2] + 2 * massA * vecA_Para[2]) / (massA + massB)

        result = [vecA_Para_after_x,vecA_Para_after_y,vecA_Para_after_z,vecB_Para_after_x,vecB_Para_after_y,vecB_Para_after_z]
        return result