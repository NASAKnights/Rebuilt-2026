#include "utils/DeployFileUtils.h"

#include <exception>
#include <fstream>
#include <sstream>

#include <frc/Errors.h>
#include <frc/Filesystem.h>
#include <frc/RobotBase.h>

#ifdef _WIN32
#include <windows.h>
#endif

namespace utils {
namespace {

std::vector<std::string> ParseCsvLine(const std::string& line) {
  std::vector<std::string> cells;
  std::string cell;
  bool inQuotes = false;

  for (size_t i = 0; i < line.size(); ++i) {
    const char ch = line[i];
    if (ch == '"') {
      if (inQuotes && i + 1 < line.size() && line[i + 1] == '"') {
        cell.push_back('"');
        ++i;
      } else {
        inQuotes = !inQuotes;
      }
    } else if (ch == ',' && !inQuotes) {
      cells.push_back(cell);
      cell.clear();
    } else {
      cell.push_back(ch);
    }
  }

  cells.push_back(cell);
  return cells;
}

std::string EscapeCsvCell(const std::string& cell) {
  const bool needsQuotes = cell.find_first_of(",\"\n\r") != std::string::npos;
  if (!needsQuotes) {
    return cell;
  }

  std::string escaped = "\"";
  for (const char ch : cell) {
    if (ch == '"') {
      escaped += "\"\"";
    } else {
      escaped.push_back(ch);
    }
  }
  escaped.push_back('"');
  return escaped;
}

}  // namespace

std::string DeployFileUtils::DeriveFileNameFromNtPath(const std::string& ntPath,
                                                      const std::string& extension) {
  std::string normalized = ntPath;
  while (!normalized.empty() && (normalized.front() == '/' || normalized.front() == '\\')) {
    normalized.erase(normalized.begin());
  }
  while (!normalized.empty() && (normalized.back() == '/' || normalized.back() == '\\')) {
    normalized.pop_back();
  }

  for (char& ch : normalized) {
    if (ch == '/' || ch == '\\') {
      ch = '_';
    }
  }

  if (normalized.empty()) {
    normalized = "map";
  }

  std::string result = normalized;
  if (!extension.empty()) {
    if (extension.starts_with('.')) {
      result += extension;
    } else {
      result.push_back('.');
      result += extension;
    }
  }

  return result;
}

std::filesystem::path DeployFileUtils::ResolveDeployFilePathFromNtPath(
    const std::string& ntPath,
    const std::string& extension) {
  return ResolveDeployFilePath(DeriveFileNameFromNtPath(ntPath, extension));
}

std::filesystem::path DeployFileUtils::ResolveDeployFilePath(const std::string& fileName) {
  if constexpr (frc::RobotBase::IsSimulation()) {
    try {
      auto current = std::filesystem::current_path();
      while (true) {
        auto candidateDeployDir = current / "src" / "main" / "deploy";
        if (std::filesystem::exists(candidateDeployDir)) {
          return candidateDeployDir / fileName;
        }

        if (!current.has_parent_path() || current == current.parent_path()) {
          break;
        }
        current = current.parent_path();
      }
    } catch (const std::exception&) {
      // Fall back to WPILib's deploy directory below.
    }
  }

  return std::filesystem::path{frc::filesystem::GetDeployDirectory()} / fileName;
}

bool DeployFileUtils::LoadCsvFile(const std::filesystem::path& filePath,
                                  std::vector<std::vector<std::string>>& rows) {
  rows.clear();

  try {
    std::string fileContents;
    if (!LoadFile(filePath, fileContents)) {
      return false;
    }

    std::istringstream stream(fileContents);
    std::string line;
    while (std::getline(stream, line)) {
      if (!line.empty()) {
        rows.push_back(ParseCsvLine(line));
      }
    }

    return !rows.empty();
  } catch (const std::exception& e) {
    FRC_ReportWarning("Failed to load CSV file '{}': {}", filePath.string(), e.what());
  } catch (...) {
    FRC_ReportWarning("Failed to load CSV file '{}': unknown error", filePath.string());
  }

  rows.clear();
  return false;
}

bool DeployFileUtils::LoadFile(const std::filesystem::path& filePath,
                               std::string& contents) {
  contents.clear();

  try {
    if (filePath.empty() || !std::filesystem::exists(filePath)) {
      if (!filePath.empty()) {
        FRC_ReportWarning("File not found: {}", filePath.string());
      }
      return false;
    }

    std::ifstream file(filePath, std::ios::binary);
    if (!file.is_open()) {
      FRC_ReportWarning("Unable to open file: {}", filePath.string());
      return false;
    }

    std::ostringstream buffer;
    buffer << file.rdbuf();
    contents = buffer.str();
    return true;
  } catch (const std::exception& e) {
    FRC_ReportWarning("Failed to load file '{}': {}", filePath.string(), e.what());
  } catch (...) {
    FRC_ReportWarning("Failed to load file '{}': unknown error", filePath.string());
  }

  contents.clear();
  return false;
}

bool DeployFileUtils::SaveCsvFile(const std::filesystem::path& filePath,
                                  const std::vector<std::vector<std::string>>& rows) {
  try {
    if (filePath.has_parent_path()) {
      std::filesystem::create_directories(filePath.parent_path());
    }

    std::filesystem::path tempPath = filePath;
    tempPath += ".tmp";
    if (std::filesystem::exists(tempPath)) {
      std::filesystem::remove(tempPath);
    }

    std::ofstream file(tempPath, std::ios::out | std::ios::trunc);
    if (!file.is_open()) {
      FRC_ReportWarning("Unable to save CSV file: {}", filePath.string());
      return false;
    }

    for (const auto& row : rows) {
      for (size_t i = 0; i < row.size(); ++i) {
        if (i > 0) {
          file << ",";
        }
        file << EscapeCsvCell(row[i]);
      }
      file << "\n";
    }

    file.flush();
    if (!file.good()) {
      FRC_ReportWarning("Failed to flush CSV file: {}", filePath.string());
      file.close();
      std::filesystem::remove(tempPath);
      return false;
    }

    file.close();

#ifdef _WIN32
    if (!MoveFileExW(tempPath.c_str(), filePath.c_str(),
                     MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
      std::filesystem::remove(tempPath);
      FRC_ReportWarning("Failed to replace CSV file: {}", filePath.string());
      return false;
    }
#else
    std::filesystem::rename(tempPath, filePath);
#endif
    return true;
  } catch (const std::exception& e) {
    try {
      std::filesystem::path tempPath = filePath;
      tempPath += ".tmp";
      if (std::filesystem::exists(tempPath)) {
        std::filesystem::remove(tempPath);
      }
    } catch (...) {
      // Best-effort cleanup only.
    }
    FRC_ReportWarning("Failed to save CSV file '{}': {}", filePath.string(), e.what());
  } catch (...) {
    try {
      std::filesystem::path tempPath = filePath;
      tempPath += ".tmp";
      if (std::filesystem::exists(tempPath)) {
        std::filesystem::remove(tempPath);
      }
    } catch (...) {
      // Best-effort cleanup only.
    }
    FRC_ReportWarning("Failed to save CSV file '{}': unknown error", filePath.string());
  }

  return false;
}

}  // namespace utils
