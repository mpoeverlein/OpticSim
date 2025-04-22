#ifndef CONFIG_LOADER_HPP
#define CONFIG_LOADER_HPP

#include <string>
#include <unordered_map>

class ConfigLoader {
public:
    static void loadFromFile(const std::string& filename);
};

#endif /* CONFIG_LOADER_HPP */