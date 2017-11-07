#ifndef ROSPACKAGEHELPER_H
#define ROSPACKAGEHELPER_H

#ifndef WIN_VREP
#include <iostream>
#include <map>

/*!
 * \brief A helper class that resolves the root directory for a "package://"-schemed URLs in URDF mesh definitions.
 *
 * Internally, it uses the Bash "rospack find" command, so in order for this to work, you need to run V-REP from
 * a ROS-enabled command line.
 */
class rosPackageHelper
{
public:
    /*!
     * \brief Get the root directory for the packageName, so that 'result + "/" + packageName' is the absolute path to the package.
     * \param packageName Name of the package to get root directory for.
     * \return An absolute path to the package's root directory.
     */
    static std::string getPackageRootDir(std::string packageName);
private:
    /*!
     * \brief A cache for the "rospack find" results.
     *
     * It is really needed since rospack is pretty slow on larger workspaces.
     */
    static std::map<std::string, std::string> rospackCache;

    /*!
     * \brief Execute a string in Bash and return its stdout as a string.
     *
     * This is a dangerous function. In this particular case, it could have been used to attack the computer
     * by supplying an adverse package name. Always be sure what you're doing.
     * \param command The command to execute.
     * \return Stdout of the command.
     * \throw std::runtime_exception If the execution failed (the command returned a non-zero exit code).
     */
    static std::string exec(const char* command);
};
#endif
#endif // ROSPACKAGEHELPER_H
