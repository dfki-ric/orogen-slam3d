#ifndef SLAM3D_DATABASE_CONFIGURATION_HPP
#define SLAM3D_DATABASE_CONFIGURATION_HPP

#include <string>

namespace slam3d
{
	struct Neo4jConfig
	{
        ServerConfig() : host("127.0.0.1"), port(7687), user("neo4j"), passwd("neo4j") {}

        std::string host;
        int port;
        std::string user;
        std::string passwd;
	};

    struct RedisConfig
    {
        RedisConfig() : host("127.0.0.1"), port(6379), cacheSize(0), useBinaryArchive(false) {}

        std::string host;
        int port;
        size_t cacheSize;
        bool useBinaryArchive;
    };
}

#endif
