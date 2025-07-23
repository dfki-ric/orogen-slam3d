#ifndef SLAM3D_DATABASE_CONFIGURATION_HPP
#define SLAM3D_DATABASE_CONFIGURATION_HPP

#include <string>

namespace slam3d
{
	struct Neo4jConfig
	{
        Neo4jConfig() : neo4j_host("127.0.0.1"), neo4j_port(7687), user("neo4j"), passwd("neo4j") {}

        std::string neo4j_host;
        int neo4j_port;
        std::string user;
        std::string passwd;
	};

    struct RedisConfig
    {
        RedisConfig() : redis_host("127.0.0.1"), redis_port(6379), cacheSize(0), useBinaryArchive(false) {}

        std::string redis_host;
        int redis_port;
        size_t cacheSize;
        bool useBinaryArchive;
    };
}

#endif
