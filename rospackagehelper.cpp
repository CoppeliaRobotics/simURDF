#include "rospackagehelper.h"

#ifndef WIN_VREP
#include <cstdio>
#include <sstream>
#include <stdexcept>
#include <sys/wait.h>

#include "commonFunctions.h"

std::map<std::string, std::string> rosPackageHelper::rospackCache;

std::string rosPackageHelper::getPackageRootDir(std::string packageName)
{
    // check the cache
    if (rospackCache.find(packageName) != rospackCache.end())
        return rospackCache[packageName];

    // construct the rospack command
    std::string rospackFind = std::string("rospack find ") + packageName;
    try {
        // execute the rospack
        printToConsole(rospackFind.c_str());
        std::string packagePath = exec(rospackFind.c_str());

        if (packagePath.length() > 0) {
            // get rid of the last path component (the package name)
            size_t packagePathLastComponentPos = packagePath.find_last_of("/");
            if (packagePathLastComponentPos == packagePath.length()) // the path ends with a slash
                packagePathLastComponentPos = packagePath.find_last_of("/", packagePath.length()-1);

            std::string packageRootDir = packagePath.substr(0, packagePathLastComponentPos);
            printToConsole((
                std::string("Package root for ") + packageName + std::string(" is ") + packageRootDir
            ).c_str());

            // save the found root dir to the cache
            rospackCache[packageName] = packageRootDir;
            return packageRootDir;
        } else {
            printToConsole("Warning: rospack returned empty package path.");
            return std::string("");
        }
    } catch (std::runtime_error& e) {
        printToConsole((std::string("rospack failed: ") + std::string(e.what())).c_str());
        return std::string("");
    }
}

std::string rosPackageHelper::exec(const char* command) {
    // open a pipe to the command execution in Bash
    FILE* pipe = popen(command, "r");
    if (!pipe) {
        std::stringstream ss;
        ss << "Command '" << command << "' failed to execute.";
        throw std::runtime_error(ss.str());
    }

    // read the stdout
    char buffer[128];
    std::string result = "";
    while(!std::feof(pipe)) {
        if(std::fgets(buffer, 128, pipe) != NULL)
            result += buffer;
    }

    // check the exit status
    const int commandResult = pclose(pipe);
    const int exitCode = WEXITSTATUS(commandResult);
    if (exitCode != 0) {
        std::stringstream ss;
        ss << "Command '" << command << "' exited with status " << exitCode << ".";
        throw std::runtime_error(ss.str());
    }

    // trim the last newline and other whitespace
    const std::string WHITESPACE = " \n\r\t";
    const size_t lastNonWhitespacePos = result.find_last_not_of(WHITESPACE);

    return (lastNonWhitespacePos == std::string::npos) ? "" : result.substr(0, lastNonWhitespacePos+1);
}
#endif
