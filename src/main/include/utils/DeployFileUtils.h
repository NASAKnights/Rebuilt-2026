#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace utils {

class DeployFileUtils {
 public:
  /**
   * Resolves a file path inside the robot deploy directory.
   *
   * On a real roboRIO, this returns WPILib's deploy directory plus fileName.
   * In simulation, it first walks up from the current working directory looking
   * for this repo's src/main/deploy directory, then falls back to WPILib's
   * deploy directory if the repo path cannot be found.
   */
  static std::filesystem::path ResolveDeployFilePath(const std::string& fileName);

  /**
   * Derives a deploy file name from a NetworkTables path.
   *
   * The path is normalized by stripping leading/trailing slashes and replacing
   * path separators with underscores. For example, "LaunchCalculator/Points"
   * becomes "LaunchCalculator_Points.csv".
   */
  static std::string DeriveFileNameFromNtPath(const std::string& ntPath,
                                              const std::string& extension = ".csv");

  /**
   * Resolves a deploy file path derived from a NetworkTables path.
   *
   * This combines DeriveFileNameFromNtPath() with ResolveDeployFilePath().
   */
  static std::filesystem::path ResolveDeployFilePathFromNtPath(
      const std::string& ntPath,
      const std::string& extension = ".csv");

  /**
   * Loads a CSV file into rows of string cells.
   *
   * @return true when the file exists, opens, and is parsed into at least one
   *         row. The first returned row is normally the header.
   */
  static bool LoadCsvFile(const std::filesystem::path& filePath,
                          std::vector<std::vector<std::string>>& rows);

  /**
   * Loads a file into a string without applying any format-specific parsing.
   *
   * The file is opened in binary mode so the returned string preserves bytes
   * exactly as stored on disk. An empty file is considered a successful load.
   */
  static bool LoadFile(const std::filesystem::path& filePath,
                       std::string& contents);

  /**
   * Saves rows of string cells to a CSV file.
   *
   * Parent directories are created automatically when needed.
   */
  static bool SaveCsvFile(const std::filesystem::path& filePath,
                          const std::vector<std::vector<std::string>>& rows);
};

}  // namespace utils
