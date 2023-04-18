#ifndef USEFULL_SERIALIZATIONS_H
#define USEFULL_SERIALIZATIONS_H

#include <glm/glm.hpp>

namespace glm
{
    template<class Archive>
    void serialize(Archive & archive,
                glm::vec3 & m)
    {
        archive( m.x, m.y, m.z );
    }

    template<class Archive>
    void serialize(Archive & archive,
                glm::vec2 & m)
    {
        archive(m.x, m.y);
    }

    template<class Archive>
    void serialize(Archive & archive,
                glm::ivec3 & m)
    {
        archive( m.x, m.y, m.z );
    }

    template<class Archive>
    void serialize(Archive & archive,
                glm::mat3 & m)
    {
        archive(m[0], m[1], m[2]);
    }
}

#endif