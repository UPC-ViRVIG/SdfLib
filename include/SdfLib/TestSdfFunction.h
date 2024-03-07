#ifndef TEST_SDF_FUNCTION_H
#define TEST_SDF_FUNCTION_H

#include "SdfFunction.h"
#include <glm/matrix.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

class TestSdfFunction : public sdflib::SdfFunction
{
public:
    TestSdfFunction() : mesh1("../../models/SimpleTemplePart1.ply"), mesh2("../../models/SimpleTemplePart2.ply"), mesh3("../../models/SimpleTemplePart3.ply")
    {
        auto normalizeMesh = [](sdflib::Mesh& mesh)
        {
            // Normalize model units
            const sdflib::BoundingBox box = mesh.getBoundingBox();
            const glm::vec3 boxSize = box.getSize();
            const float maxSize = glm::max(glm::max(boxSize.x, boxSize.y), boxSize.z);
            mesh.applyTransform(glm::scale(glm::mat4(1.0), glm::vec3(2.0f/maxSize)) *
                                glm::translate(glm::mat4(1.0), -box.getCenter()));
        };

        // normalizeMesh(mesh1);
        meshSdf1 = std::unique_ptr<sdflib::MeshSvhSdf>(new sdflib::MeshSvhSdf(mesh1));

        // normalizeMesh(mesh2);
        meshSdf2 = std::unique_ptr<sdflib::MeshSvhSdf>(new sdflib::MeshSvhSdf(mesh2));

        // normalizeMesh(mesh3);
        meshSdf3 = std::unique_ptr<sdflib::MeshSvhSdf>(new sdflib::MeshSvhSdf(mesh3));
    }

    float getDistance(glm::vec3 sample) const override
    {
        sample.y -= 0.01;
        // auto box = [&](glm::vec3 boxPos, glm::vec3 boxSize, float r)
        // {
        //     glm::vec3 q = glm::abs(sample - boxPos) - boxSize;
        //     return glm::length(glm::max(q, glm::vec3(0.0f))) + glm::min(glm::max(q.x, glm::max(q.y, q.z)), 0.0f) - r;
        // };

        // auto sphere = [&](glm::vec3 spherePos, float r)
        // {
        //     return glm::length(sample - spherePos) - r;
        // };

        // auto cylinder = [&](glm::vec3 cylPos, float h, float r)
        // {
        //     glm::vec3 p = sample - cylPos;
        //     glm::vec2 d = glm::abs(glm::vec2(glm::length(glm::vec2(p.x, p.z)),p.y)) - glm::vec2(r,h);
        //     return glm::min(glm::max(d.x,d.y),0.0f) + glm::length(glm::max(d, glm::vec2(0.0f)));
        // };

        // auto smoothUnion = [](float d1, float d2, float k)
        // {
        //     float h = glm::clamp( 0.5f + 0.5f*(d2-d1)/k, 0.0f, 1.0f );
        //     return glm::mix( d2, d1, h ) - k*h*(1.0f-h);
        //     //return glm::min(d1, d2);
        // };


        // float down1 = box(glm::vec3(0.0, -0.9, 0.0), glm::vec3(0.85, 0.05, 0.85), 0.01);
        // float down2 = box(glm::vec3(0.0, -0.8, -0.08), glm::vec3(0.85, 0.05, 0.77), 0.01);
        // float down3 = box(glm::vec3(0.0, -0.7, -0.16), glm::vec3(0.85, 0.05, 0.69), 0.01);
        // float down = glm::min(down1, glm::min(down2, down3));

        // float structure = glm::min(down, box(glm::vec3(0.0, 0.8, 0.0), glm::vec3(0.85, 0.1, 0.85), 0.01));

        // float columMin = 100000.0f;
        // for(uint32_t i=0; i < 3; i++)
        // {
        //     float c1 = cylinder(glm::vec3(-0.65, 0.0, 0.53 - 0.345 * (float(i) + 1.0)), 0.8, 0.13);
        //     float b1 = cylinder(glm::vec3(-0.65, -0.6, 0.53 - 0.345 * (float(i) + 1.0)), 0.05, 0.14) - 0.02;
        //     columMin = glm::min(columMin, smoothUnion(c1, b1, 0.1));

        //     float c2 = cylinder(glm::vec3(0.65, 0.0, 0.53 - 0.345 * (float(i) + 1.0)), 0.8, 0.13);
        //     float b2 = cylinder(glm::vec3(0.65, -0.6, 0.53 - 0.345 * (float(i) + 1.0)), 0.05, 0.14) - 0.02;
        //     columMin = glm::min(columMin, smoothUnion(c2, b2, 0.1));
        // }

        // float armadillo = meshSdf1->getDistance(glm::vec3(1.0f, 1.0f, -1.0f) * (1.8f * (sample - glm::vec3(0.0, -0.68, -0.5)) + glm::vec3(0.0, -1.0, 0.0))) / 1.8f;
        // // float m2 = meshSdf1->getDistance(10.0f * (sample - glm::vec3(0.8, 0.5, 0.5))) / 10.0f;

        // float s = smoothUnion(sphere(glm::vec3(-0.1f, -0.4f, 0.2f), 0.1f), sphere(glm::vec3(0.1f, -0.4f, 0.2f), 0.1f), 0.1);
        // // float s = meshSdf2->getDistance(5.0f * (sample - glm::vec3(0.0f, -0.4f, 0.2f))) / 5.0f;;

        // // return glm::min(glm::min(smoothUnion(columMin, structure, 0.04), armadillo), s);
        // return s;

        float roof = meshSdf3->getDistance(sample) - 0.08f;
        return glm::min(roof, glm::min(meshSdf1->getDistance(sample), meshSdf2->getDistance(sample)));
    }

    float getDistance(glm::vec3 sample, glm::vec3& outGradient) const override
    {
        sample.y -= 0.01;
        glm::vec3 outG1, outG2, outG3;
        const float epsilon = 4e-4;
        outGradient = glm::vec3(
                        meshSdf3->getDistance(sample + glm::vec3(epsilon, 0, 0)) - meshSdf3->getDistance(sample - glm::vec3(epsilon, 0, 0)),
                        meshSdf3->getDistance(sample + glm::vec3(0, epsilon, 0)) - meshSdf3->getDistance(sample - glm::vec3(0, epsilon, 0)),
                        meshSdf3->getDistance(sample + glm::vec3(0, 0, epsilon)) - meshSdf3->getDistance(sample - glm::vec3(0, 0, epsilon)));
        outG3 = glm::normalize(outGradient);

        // if(glm::isnan(outGradient.x) || glm::isnan(outGradient.y) || glm::isnan(outGradient.z)) 
        // {
        //     outGradient = glm::vec3(1.0, 0.0, 0.0);
        //     std::cout << "out gradient" << std::endl;
        // }

        // return dist;

        // return meshSdf3->getDistance(sample, outGradient);

        float roof = meshSdf3->getDistance(sample) - 0.08f;
        float p1 = meshSdf1->getDistance(sample, outG1);
        float p2 = meshSdf2->getDistance(sample, outG2);
        
        if(p1 < roof)
        {
            if(p1 < p2) outGradient = outG1;
            else outGradient = outG2;
        }
        else
        {
            if(roof < p2) outGradient = outG3;
            else outGradient = outG2;
        }
        return glm::min(roof, glm::min(p1, p2));
    }
private:
    sdflib::Mesh mesh1;
    std::unique_ptr<sdflib::MeshSvhSdf> meshSdf1;
    sdflib::Mesh mesh2;
    std::unique_ptr<sdflib::MeshSvhSdf> meshSdf2;
    sdflib::Mesh mesh3;
    std::unique_ptr<sdflib::MeshSvhSdf> meshSdf3;
};

class TestSdfFunction2 : public sdflib::SdfFunction
{
public:
    TestSdfFunction2() : mesh1("../../models/TreesPart1.ply"), mesh2("../../models/TreesPart2.ply")
    {
        auto normalizeMesh = [](sdflib::Mesh& mesh)
        {
            // Normalize model units
            const sdflib::BoundingBox box = mesh.getBoundingBox();
            const glm::vec3 boxSize = box.getSize();
            const float maxSize = glm::max(glm::max(boxSize.x, boxSize.y), boxSize.z);
            mesh.applyTransform(glm::scale(glm::mat4(1.0), glm::vec3(2.0f/maxSize)) *
                                glm::translate(glm::mat4(1.0), -box.getCenter()));
        };

        // normalizeMesh(mesh1);
        meshSdf1 = std::unique_ptr<sdflib::MeshSvhSdf>(new sdflib::MeshSvhSdf(mesh1));

        // normalizeMesh(mesh2);
        meshSdf2 = std::unique_ptr<sdflib::MeshSvhSdf>(new sdflib::MeshSvhSdf(mesh2));

        // normalizeMesh(mesh3);
        // meshSdf3 = std::unique_ptr<sdflib::MeshSvhSdf>(new sdflib::MeshSvhSdf(mesh3));
    }

    float getDistance(glm::vec3 sample) const override
    {
        // auto box = [&](glm::vec3 boxPos, glm::vec3 boxSize, float r)
        // {
        //     glm::vec3 q = glm::abs(sample - boxPos) - boxSize;
        //     return glm::length(glm::max(q, glm::vec3(0.0f))) + glm::min(glm::max(q.x, glm::max(q.y, q.z)), 0.0f) - r;
        // };

        // auto sphere = [&](glm::vec3 spherePos, float r)
        // {
        //     return glm::length(sample - spherePos) - r;
        // };

        // auto cylinder = [&](glm::vec3 cylPos, float h, float r)
        // {
        //     glm::vec3 p = sample - cylPos;
        //     glm::vec2 d = glm::abs(glm::vec2(glm::length(glm::vec2(p.x, p.z)),p.y)) - glm::vec2(r,h);
        //     return glm::min(glm::max(d.x,d.y),0.0f) + glm::length(glm::max(d, glm::vec2(0.0f)));
        // };

        // auto smoothUnion = [](float d1, float d2, float k)
        // {
        //     float h = glm::clamp( 0.5f + 0.5f*(d2-d1)/k, 0.0f, 1.0f );
        //     return glm::mix( d2, d1, h ) - k*h*(1.0f-h);
        //     //return glm::min(d1, d2);
        // };


        // float down1 = box(glm::vec3(0.0, -0.9, 0.0), glm::vec3(0.85, 0.05, 0.85), 0.01);
        // float down2 = box(glm::vec3(0.0, -0.8, -0.08), glm::vec3(0.85, 0.05, 0.77), 0.01);
        // float down3 = box(glm::vec3(0.0, -0.7, -0.16), glm::vec3(0.85, 0.05, 0.69), 0.01);
        // float down = glm::min(down1, glm::min(down2, down3));

        // float structure = glm::min(down, box(glm::vec3(0.0, 0.8, 0.0), glm::vec3(0.85, 0.1, 0.85), 0.01));

        // float columMin = 100000.0f;
        // for(uint32_t i=0; i < 3; i++)
        // {
        //     float c1 = cylinder(glm::vec3(-0.65, 0.0, 0.53 - 0.345 * (float(i) + 1.0)), 0.8, 0.13);
        //     float b1 = cylinder(glm::vec3(-0.65, -0.6, 0.53 - 0.345 * (float(i) + 1.0)), 0.05, 0.14) - 0.02;
        //     columMin = glm::min(columMin, smoothUnion(c1, b1, 0.1));

        //     float c2 = cylinder(glm::vec3(0.65, 0.0, 0.53 - 0.345 * (float(i) + 1.0)), 0.8, 0.13);
        //     float b2 = cylinder(glm::vec3(0.65, -0.6, 0.53 - 0.345 * (float(i) + 1.0)), 0.05, 0.14) - 0.02;
        //     columMin = glm::min(columMin, smoothUnion(c2, b2, 0.1));
        // }

        // float armadillo = meshSdf1->getDistance(glm::vec3(1.0f, 1.0f, -1.0f) * (1.8f * (sample - glm::vec3(0.0, -0.68, -0.5)) + glm::vec3(0.0, -1.0, 0.0))) / 1.8f;
        // // float m2 = meshSdf1->getDistance(10.0f * (sample - glm::vec3(0.8, 0.5, 0.5))) / 10.0f;

        // float s = smoothUnion(sphere(glm::vec3(-0.1f, -0.4f, 0.2f), 0.1f), sphere(glm::vec3(0.1f, -0.4f, 0.2f), 0.1f), 0.1);
        // // float s = meshSdf2->getDistance(5.0f * (sample - glm::vec3(0.0f, -0.4f, 0.2f))) / 5.0f;;

        // // return glm::min(glm::min(smoothUnion(columMin, structure, 0.04), armadillo), s);
        // return s;

        return glm::min(meshSdf1->getDistance(sample) - 0.04f, meshSdf2->getDistance(sample));
    }

    float getDistance(glm::vec3 sample, glm::vec3& outGradient) const override
    {
        glm::vec3 outG1, outG2, outG3;
        const float epsilon = 4e-4;
        outGradient = glm::vec3(
                        getDistance(sample + glm::vec3(epsilon, 0, 0)) - getDistance(sample - glm::vec3(epsilon, 0, 0)),
                        getDistance(sample + glm::vec3(0, epsilon, 0)) - getDistance(sample - glm::vec3(0, epsilon, 0)),
                        getDistance(sample + glm::vec3(0, 0, epsilon)) - getDistance(sample - glm::vec3(0, 0, epsilon)));
        outGradient = glm::normalize(outGradient);
        return getDistance(sample);

        // if(glm::isnan(outGradient.x) || glm::isnan(outGradient.y) || glm::isnan(outGradient.z)) 
        // {
        //     outGradient = glm::vec3(1.0, 0.0, 0.0);
        //     std::cout << "out gradient" << std::endl;
        // }

        // return dist;

        // return meshSdf3->getDistance(sample, outGradient);

        // float roof = meshSdf3->getDistance(sample) - 0.08f;
        // float p1 = meshSdf1->getDistance(sample, outG1);
        // float p2 = meshSdf2->getDistance(sample, outG2);
        
        // if(p1 < roof)
        // {
        //     if(p1 < p2) outGradient = outG1;
        //     else outGradient = outG2;
        // }
        // else
        // {
        //     if(roof < p2) outGradient = outG3;
        //     else outGradient = outG2;
        // }
        // return glm::min(roof, glm::min(p1, p2));
    }
private:
    sdflib::Mesh mesh1;
    std::unique_ptr<sdflib::MeshSvhSdf> meshSdf1;
    sdflib::Mesh mesh2;
    std::unique_ptr<sdflib::MeshSvhSdf> meshSdf2;
    sdflib::Mesh mesh3;
    std::unique_ptr<sdflib::MeshSvhSdf> meshSdf3;
};


class TestSdfFunction3 : public sdflib::SdfFunction
{
public:
    TestSdfFunction3() : mesh1("../../models/roomPart.ply"), mesh2("../../models/models/bunny.ply")
    {
        auto normalizeMesh = [](sdflib::Mesh& mesh)
        {
            // Normalize model units
            const sdflib::BoundingBox box = mesh.getBoundingBox();
            const glm::vec3 boxSize = box.getSize();
            const float maxSize = glm::max(glm::max(boxSize.x, boxSize.y), boxSize.z);
            mesh.applyTransform(glm::scale(glm::mat4(1.0), glm::vec3(2.0f/maxSize)) *
                                glm::translate(glm::mat4(1.0), -box.getCenter()));
        };

        // normalizeMesh(mesh1);
        meshSdf1 = std::unique_ptr<sdflib::MeshSvhSdf>(new sdflib::MeshSvhSdf(mesh1));

        normalizeMesh(mesh2);
        meshSdf2 = std::unique_ptr<sdflib::MeshSvhSdf>(new sdflib::MeshSvhSdf(mesh2));

        // normalizeMesh(mesh3);
        // meshSdf3 = std::unique_ptr<sdflib::MeshSvhSdf>(new sdflib::MeshSvhSdf(mesh3));
    }

    float getDistance(glm::vec3 sample) const override
    {
        sample.y += 0.02;

        auto box = [&](glm::vec3 boxPos, glm::vec3 boxSize, float r)
        {
            glm::vec3 q = glm::abs(sample - boxPos) - boxSize;
            return glm::length(glm::max(q, glm::vec3(0.0f))) + glm::min(glm::max(q.x, glm::max(q.y, q.z)), 0.0f) - r;
        };

        auto sphere = [&](glm::vec3 spherePos, float r)
        {
            return glm::length(sample - spherePos) - r;
        };

        // auto cylinder = [&](glm::vec3 cylPos, float h, float r)
        // {
        //     glm::vec3 p = sample - cylPos;
        //     glm::vec2 d = glm::abs(glm::vec2(glm::length(glm::vec2(p.x, p.z)),p.y)) - glm::vec2(r,h);
        //     return glm::min(glm::max(d.x,d.y),0.0f) + glm::length(glm::max(d, glm::vec2(0.0f)));
        // };

        // auto smoothUnion = [](float d1, float d2, float k)
        // {
        //     float h = glm::clamp( 0.5f + 0.5f*(d2-d1)/k, 0.0f, 1.0f );
        //     return glm::mix( d2, d1, h ) - k*h*(1.0f-h);
        //     //return glm::min(d1, d2);
        // };


        // float down1 = box(glm::vec3(0.0, -0.9, 0.0), glm::vec3(0.85, 0.05, 0.85), 0.01);
        // float down2 = box(glm::vec3(0.0, -0.8, -0.08), glm::vec3(0.85, 0.05, 0.77), 0.01);
        // float down3 = box(glm::vec3(0.0, -0.7, -0.16), glm::vec3(0.85, 0.05, 0.69), 0.01);
        // float down = glm::min(down1, glm::min(down2, down3));

        // float structure = glm::min(down, box(glm::vec3(0.0, 0.8, 0.0), glm::vec3(0.85, 0.1, 0.85), 0.01));

        // float columMin = 100000.0f;
        // for(uint32_t i=0; i < 3; i++)
        // {
        //     float c1 = cylinder(glm::vec3(-0.65, 0.0, 0.53 - 0.345 * (float(i) + 1.0)), 0.8, 0.13);
        //     float b1 = cylinder(glm::vec3(-0.65, -0.6, 0.53 - 0.345 * (float(i) + 1.0)), 0.05, 0.14) - 0.02;
        //     columMin = glm::min(columMin, smoothUnion(c1, b1, 0.1));

        //     float c2 = cylinder(glm::vec3(0.65, 0.0, 0.53 - 0.345 * (float(i) + 1.0)), 0.8, 0.13);
        //     float b2 = cylinder(glm::vec3(0.65, -0.6, 0.53 - 0.345 * (float(i) + 1.0)), 0.05, 0.14) - 0.02;
        //     columMin = glm::min(columMin, smoothUnion(c2, b2, 0.1));
        // }

        // float armadillo = meshSdf1->getDistance(glm::vec3(1.0f, 1.0f, -1.0f) * (1.8f * (sample - glm::vec3(0.0, -0.68, -0.5)) + glm::vec3(0.0, -1.0, 0.0))) / 1.8f;
        // // float m2 = meshSdf1->getDistance(10.0f * (sample - glm::vec3(0.8, 0.5, 0.5))) / 10.0f;

        // float s = smoothUnion(sphere(glm::vec3(-0.1f, -0.4f, 0.2f), 0.1f), sphere(glm::vec3(0.1f, -0.4f, 0.2f), 0.1f), 0.1);
        // // float s = meshSdf2->getDistance(5.0f * (sample - glm::vec3(0.0f, -0.4f, 0.2f))) / 5.0f;;

        // // return glm::min(glm::min(smoothUnion(columMin, structure, 0.04), armadillo), s);
        // return s;

        auto sub = [](float d1, float d2)
        {
            return glm::max(-d2, d1);
        };

        auto subSmooth = []( float d2, float d1, float k )
        {
            float h = glm::clamp( 0.5 - 0.5*(d2+d1)/k, 0.0, 1.0 );
            return glm::mix( d2, -d1, h ) + k*h*(1.0-h);
        };

        float room = meshSdf1->getDistance(sample) - 0.01f;
        float spheres = sphere(glm::vec3(0.1f, 0.2f, -0.1f), 0.15);
        spheres = glm::min(spheres, sphere(glm::vec3(-0.1f, 0.2, 0.1f), 0.2));
        spheres = glm::min(spheres, sphere(glm::vec3(-0.4, -0.2, 0.0), 0.15));
        spheres = glm::min(spheres, sphere(glm::vec3(0.0, -0.2, 0.4), 0.3));
        spheres = glm::min(spheres, sphere(glm::vec3(0.0, -0.2, -0.4), 0.3));

        float mbox = subSmooth(box(glm::vec3(0.0f, -0.2f, 0.0f), glm::vec3(0.35f), 0.01f), spheres, 0.02f);

        return glm::min(room, mbox);
    }

    float getDistance(glm::vec3 sample, glm::vec3& outGradient) const override
    {
        sample.y += 0.02;
        glm::vec3 outG1, outG2, outG3;
        const float epsilon = 4e-4;
        outGradient = glm::vec3(
                        getDistance(sample + glm::vec3(epsilon, 0, 0)) - getDistance(sample - glm::vec3(epsilon, 0, 0)),
                        getDistance(sample + glm::vec3(0, epsilon, 0)) - getDistance(sample - glm::vec3(0, epsilon, 0)),
                        getDistance(sample + glm::vec3(0, 0, epsilon)) - getDistance(sample - glm::vec3(0, 0, epsilon)));
        outGradient = glm::normalize(outGradient);
        

        if(glm::isnan(outGradient.x) || glm::isnan(outGradient.y) || glm::isnan(outGradient.z)) 
        {
            outGradient = glm::vec3(1.0, 0.0, 0.0);
            std::cout << "out gradient" << std::endl;
        }

        return getDistance(sample);

        // return meshSdf3->getDistance(sample, outGradient);

        // float roof = meshSdf3->getDistance(sample) - 0.08f;
        // float p1 = meshSdf1->getDistance(sample, outG1);
        // float p2 = meshSdf2->getDistance(sample, outG2);
        
        // if(p1 < roof)
        // {
        //     if(p1 < p2) outGradient = outG1;
        //     else outGradient = outG2;
        // }
        // else
        // {
        //     if(roof < p2) outGradient = outG3;
        //     else outGradient = outG2;
        // }
        // return glm::min(roof, glm::min(p1, p2));
    }
private:
    sdflib::Mesh mesh1;
    std::unique_ptr<sdflib::MeshSvhSdf> meshSdf1;
    sdflib::Mesh mesh2;
    std::unique_ptr<sdflib::MeshSvhSdf> meshSdf2;
    sdflib::Mesh mesh3;
    std::unique_ptr<sdflib::MeshSvhSdf> meshSdf3;
};



#endif