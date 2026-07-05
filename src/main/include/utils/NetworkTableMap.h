#pragma once

#include "utils/DeployFileUtils.h"

#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>

#include <array>
#include <cstdio>
#include <exception>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include <frc/Errors.h>

/**
 * Converts row keys between the C++ type, NetworkTables subtable names, and
 * CSV text. Specialize this when adding a new key type.
 */
template <typename Key>
struct NetworkTableMapKeyTraits;

/** Numeric row-key formatting for distance-like tables. */
template <>
struct NetworkTableMapKeyTraits<double> {
  static std::string ToNetworkTableKey(double key) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.1f", key);
    return buf;
  }

  static std::string ToCsvKey(double key) {
    return ToNetworkTableKey(key);
  }

  static std::optional<double> FromString(const std::string& text) {
    try {
      size_t parsed = 0;
      const double value = std::stod(text, &parsed);
      if (parsed == text.size()) {
        return value;
      }
    } catch (const std::exception&) {
    }
    return std::nullopt;
  }
};

/** String row-key formatting for named states, waypoints, and actions. */
template <>
struct NetworkTableMapKeyTraits<std::string> {
  static std::string ToNetworkTableKey(const std::string& key) {
    return key;
  }

  static std::string ToCsvKey(const std::string& key) {
    return key;
  }

  static std::optional<std::string> FromString(const std::string& text) {
    if (text.empty()) {
      return std::nullopt;
    }
    return text;
  }
};

/** Integer row-key formatting for enumerated states or IDs. */
template <>
struct NetworkTableMapKeyTraits<int> {
  static std::string ToNetworkTableKey(int key) {
    return std::to_string(key);
  }

  static std::string ToCsvKey(int key) {
    return std::to_string(key);
  }

  static std::optional<int> FromString(const std::string& text) {
    try {
      size_t parsed = 0;
      const int value = std::stoi(text, &parsed);
      if (parsed == text.size()) {
        return value;
      }
    } catch (const std::exception&) {
    }
    return std::nullopt;
  }
};

/** Boolean row-key formatting for two-state groups. */
template <>
struct NetworkTableMapKeyTraits<bool> {
  static std::string ToNetworkTableKey(bool key) {
    return key ? "true" : "false";
  }

  static std::string ToCsvKey(bool key) {
    return ToNetworkTableKey(key);
  }

  static std::optional<bool> FromString(const std::string& text) {
    if (text == "true" || text == "1") {
      return true;
    }
    if (text == "false" || text == "0") {
      return false;
    }
    return std::nullopt;
  }
};

namespace ntmap_detail {

template <typename ValueType>
struct NetworkTableMapValueTraits;

template <typename T>
struct NetworkTableMapIsSupportedValue : std::false_type {};

template <>
struct NetworkTableMapValueTraits<double> {
  using ValueT = double;
  using TopicType = nt::DoubleTopic;
  using PublisherType = nt::DoublePublisher;
  using SubscriberType = nt::DoubleSubscriber;

  static TopicType GetTopic(const std::shared_ptr<nt::NetworkTable>& table,
                            const std::string& name) {
    return table->GetDoubleTopic(name);
  }

  static PublisherType Publish(TopicType& topic) {
    return topic.Publish({});
  }

  static SubscriberType Subscribe(TopicType& topic, double defaultValue) {
    return topic.Subscribe(defaultValue);
  }

  static void Write(PublisherType& publisher, double value) {
    publisher.Set(value);
  }

  static double Read(const SubscriberType& subscriber) {
    return subscriber.Get();
  }

  static std::string ToCsvString(double value) {
    std::ostringstream stream;
    stream << value;
    return stream.str();
  }

  static std::optional<double> FromString(const std::string& text) {
    try {
      size_t parsed = 0;
      const double value = std::stod(text, &parsed);
      if (parsed == text.size()) {
        return value;
      }
    } catch (const std::exception&) {
    }
    return std::nullopt;
  }
};

template <>
struct NetworkTableMapIsSupportedValue<double> : std::true_type {};

template <>
struct NetworkTableMapValueTraits<int> {
  using ValueT = int;
  using TopicType = nt::IntegerTopic;
  using PublisherType = nt::IntegerPublisher;
  using SubscriberType = nt::IntegerSubscriber;

  static TopicType GetTopic(const std::shared_ptr<nt::NetworkTable>& table,
                            const std::string& name) {
    return table->GetIntegerTopic(name);
  }

  static PublisherType Publish(TopicType& topic) {
    return topic.Publish({});
  }

  static SubscriberType Subscribe(TopicType& topic, int defaultValue) {
    return topic.Subscribe(defaultValue);
  }

  static void Write(PublisherType& publisher, int value) {
    publisher.Set(value);
  }

  static int Read(const SubscriberType& subscriber) {
    return static_cast<int>(subscriber.Get());
  }

  static std::string ToCsvString(int value) {
    return std::to_string(value);
  }

  static std::optional<int> FromString(const std::string& text) {
    try {
      size_t parsed = 0;
      const int value = std::stoi(text, &parsed);
      if (parsed == text.size()) {
        return value;
      }
    } catch (const std::exception&) {
    }
    return std::nullopt;
  }
};

template <>
struct NetworkTableMapIsSupportedValue<int> : std::true_type {};

template <>
struct NetworkTableMapValueTraits<bool> {
  using ValueT = bool;
  using TopicType = nt::BooleanTopic;
  using PublisherType = nt::BooleanPublisher;
  using SubscriberType = nt::BooleanSubscriber;

  static TopicType GetTopic(const std::shared_ptr<nt::NetworkTable>& table,
                            const std::string& name) {
    return table->GetBooleanTopic(name);
  }

  static PublisherType Publish(TopicType& topic) {
    return topic.Publish({});
  }

  static SubscriberType Subscribe(TopicType& topic, bool defaultValue) {
    return topic.Subscribe(defaultValue);
  }

  static void Write(PublisherType& publisher, bool value) {
    publisher.Set(value);
  }

  static bool Read(const SubscriberType& subscriber) {
    return subscriber.Get();
  }

  static std::string ToCsvString(bool value) {
    return value ? "true" : "false";
  }

  static std::optional<bool> FromString(const std::string& text) {
    if (text == "true" || text == "1") {
      return true;
    }
    if (text == "false" || text == "0") {
      return false;
    }
    return std::nullopt;
  }
};

template <>
struct NetworkTableMapIsSupportedValue<bool> : std::true_type {};

template <>
struct NetworkTableMapValueTraits<std::string> {
  using ValueT = std::string;
  using TopicType = nt::StringTopic;
  using PublisherType = nt::StringPublisher;
  using SubscriberType = nt::StringSubscriber;

  static TopicType GetTopic(const std::shared_ptr<nt::NetworkTable>& table,
                            const std::string& name) {
    return table->GetStringTopic(name);
  }

  static PublisherType Publish(TopicType& topic) {
    return topic.Publish({});
  }

  static SubscriberType Subscribe(TopicType& topic, const std::string& defaultValue) {
    return topic.Subscribe(defaultValue);
  }

  static void Write(PublisherType& publisher, const std::string& value) {
    publisher.Set(value);
  }

  static std::string Read(const SubscriberType& subscriber) {
    return subscriber.Get();
  }

  static std::string ToCsvString(const std::string& value) {
    return value;
  }

  static std::optional<std::string> FromString(const std::string& text) {
    return text;
  }
};

template <>
struct NetworkTableMapIsSupportedValue<std::string> : std::true_type {};

template <typename T>
struct NetworkTableMapIsSupportedKey : std::false_type {};

template <>
struct NetworkTableMapIsSupportedKey<double> : std::true_type {};

template <>
struct NetworkTableMapIsSupportedKey<int> : std::true_type {};

template <>
struct NetworkTableMapIsSupportedKey<bool> : std::true_type {};

template <>
struct NetworkTableMapIsSupportedKey<std::string> : std::true_type {};

}  // namespace ntmap_detail

/**
 * Stores a typed table of tuning values, mirrors it to NetworkTables, and
 * optionally persists it to CSV.
 *
 * @tparam Key Row-key type. Supported types are any type with a
 *         NetworkTableMapKeyTraits specialization. This file provides
 *         specializations for double, int, bool, and std::string.
 * @tparam RowTypes Ordered row value types. Each row is stored as
 *         std::tuple<RowTypes...> and each type must have a
 *         ntmap_detail::NetworkTableMapValueTraits specialization. Supported
 *         row value types are double, int, bool, and std::string.
 */
template <typename Key, typename... RowTypes>
class NetworkTableMap {
 public:
  static_assert(ntmap_detail::NetworkTableMapIsSupportedKey<Key>::value,
                "NetworkTableMap key type is not supported.");
  static_assert(sizeof...(RowTypes) > 0, "NetworkTableMap requires at least one row value type.");
  static_assert((ntmap_detail::NetworkTableMapIsSupportedValue<RowTypes>::value && ...),
                "NetworkTableMap row value type is not supported.");

  using RowTuple = std::tuple<RowTypes...>;
  using MapType = std::map<Key, RowTuple>;

  /** Publisher/subscriber pair for one typed parameter topic. */
  template <typename ValueType>
  struct ParamTelemetry {
    mutable typename ntmap_detail::NetworkTableMapValueTraits<ValueType>::PublisherType pub;
    mutable typename ntmap_detail::NetworkTableMapValueTraits<ValueType>::SubscriberType sub;
  };

  /** NetworkTables handles for every parameter in one row. */
  using RowTelemetry = std::tuple<ParamTelemetry<RowTypes>...>;

  /**
   * Creates a synchronized tuning table.
   *
   * @param tableName NetworkTables table path, such as
   *        "LaunchCalculator/Points".
   * @param keyColumnName First CSV column name, such as "Distance" or "State".
   * @param paramNames Ordered parameter names. These become both CSV columns
   *        and NetworkTables topic names.
   * @param filePath Optional absolute CSV path used for load/save persistence.
   *        On the robot this is usually a file inside the deploy directory.
   */
  NetworkTableMap(
      const std::string& tableName,
      const std::string& keyColumnName,
      const std::array<std::string, sizeof...(RowTypes)>& paramNames,
      const std::string& filePath = "")
      : m_table(nt::NetworkTableInstance::GetDefault().GetTable(tableName)),
        m_keyColumnName(keyColumnName),
        m_paramNames(paramNames),
        m_filePath(filePath) {
    if (!m_filePath.empty()) {
      LoadFromFile();
    }

    for (const auto& [key, values] : m_localMap) {
      (void)values;
      EnsureTelemetryForKey(key);
    }

    PublishCurrentTable();
  }

  /**
   * Returns the values for one row key.
   *
   * NetworkTables subscribers are sampled before the lookup, so the returned
   * row may include GUI/dashboard edits that have not yet been saved.
   */
  std::optional<RowTuple> Get(const Key& key) const {
    UpdateFromNetworkTables();
    auto it = m_localMap.find(key);
    if (it != m_localMap.end()) {
      return it->second;
    }
    return std::nullopt;
  }

  /**
   * Sets or creates one row in the local table and republishes the table.
   */
  void Set(const Key& key, RowTuple values) {
    UpdateFromNetworkTables();
    m_localMap[key] = std::move(values);
    EnsureTelemetryForKey(key);
    PublishCurrentTable();
  }

  /**
   * Sets or creates one row in the local table and republishes the table.
   */
  template <typename First, typename... Rest,
            std::enable_if_t<
                !(sizeof...(Rest) == 0 && std::is_same_v<RowTuple, std::decay_t<First>>),
                int> = 0>
  void Set(const Key& key, First&& first, Rest&&... values) {
    static_assert(1 + sizeof...(Rest) == sizeof...(RowTypes),
                  "Set() requires exactly one value for each row type.");
    Set(key, RowTuple{std::forward<First>(first), std::forward<Rest>(values)...});
  }

  /**
   * Returns a copy of the full table after sampling NetworkTables subscribers.
   */
  MapType GetMap() const {
    UpdateFromNetworkTables();
    return m_localMap;
  }

  /**
   * Pulls currently published/subscribed NetworkTables values into m_localMap.
   *
   * This also discovers compatible subtables that were created externally,
   * such as by a tuning GUI.
   */
  void UpdateFromNetworkTables() const {
    DiscoverNetworkTableRows();

    for (const auto& [key, rowPtr] : m_telemetryMap) {
      if (!rowPtr) continue;
      m_localMap[key] = ReadRow(*rowPtr, std::index_sequence_for<RowTypes...>{});
    }
  }

  /**
   * Publishes all locally held values to their NetworkTables topics.
   */
  void PublishCurrentTable() const {
    for (const auto& [key, rowPtr] : m_telemetryMap) {
      if (!rowPtr) continue;

      auto it = m_localMap.find(key);
      if (it != m_localMap.end()) {
        WriteRow(*rowPtr, it->second, std::index_sequence_for<RowTypes...>{});
      }
    }
  }

  /**
   * Saves the current table to filePath as CSV.
   *
   * NetworkTables values are sampled first so GUI/dashboard edits are included.
   * If filePath is empty, this is a no-op.
   */
  void SaveToFile() const {
    if (m_filePath.empty()) return;

    try {
      UpdateFromNetworkTables();

      std::vector<std::vector<std::string>> rows;
      std::vector<std::string> header{m_keyColumnName};
      for (const auto& name : m_paramNames) {
        header.push_back(name);
      }
      rows.push_back(std::move(header));

      for (const auto& [key, values] : m_localMap) {
        std::vector<std::string> row{NetworkTableMapKeyTraits<Key>::ToCsvKey(key)};
        AppendRowValues(row, values, std::index_sequence_for<RowTypes...>{});
        rows.push_back(std::move(row));
      }

      utils::DeployFileUtils::SaveCsvFile(m_filePath, rows);
    } catch (const std::exception& e) {
      FRC_ReportWarning("Failed to serialize NetworkTableMap '{}': {}", m_filePath, e.what());
    } catch (...) {
      FRC_ReportWarning("Failed to serialize NetworkTableMap '{}': unknown error", m_filePath);
    }
  }

  /**
   * Loads the table from filePath.
   *
   * @return true if a non-empty table was loaded, false if no file existed,
   *         parsing failed, or the loaded rows did not match RowTypes.
   */
  bool LoadFromFile() {
    try {
      std::vector<std::vector<std::string>> rows;
      if (m_filePath.empty() || !utils::DeployFileUtils::LoadCsvFile(m_filePath, rows)) {
        return false;
      }

      MapType loaded;
      for (size_t rowIndex = 1; rowIndex < rows.size(); ++rowIndex) {
        const auto& row = rows[rowIndex];
        if (row.size() != 1 + sizeof...(RowTypes)) {
          continue;
        }

        auto key = NetworkTableMapKeyTraits<Key>::FromString(row[0]);
        if (!key.has_value()) {
          continue;
        }

        RowTuple values;
        if (!ParseRowValues(row, values)) {
          continue;
        }

        loaded[*key] = values;
      }

      if (!loaded.empty()) {
        m_localMap = loaded;
        for (const auto& [key, values] : m_localMap) {
          (void)values;
          EnsureTelemetryForKey(key);
        }
        PublishCurrentTable();
        std::cout << "Loaded csv " << m_filePath << std::endl;
        return true;
      }
    } catch (const std::exception& e) {
      FRC_ReportWarning("Failed to parse NetworkTableMap '{}', starting empty: {}", m_filePath, e.what());
    } catch (...) {
      FRC_ReportWarning("Failed to parse NetworkTableMap '{}', starting empty: unknown error", m_filePath);
    }
    return false;
  }

 private:
  template <size_t I = 0>
  bool ParseRowValues(const std::vector<std::string>& row, RowTuple& values) const {
    if constexpr (I == sizeof...(RowTypes)) {
      return true;
    } else {
      using ValueType = std::tuple_element_t<I, RowTuple>;
      auto parsed = ntmap_detail::NetworkTableMapValueTraits<ValueType>::FromString(row[I + 1]);
      if (!parsed.has_value()) {
        return false;
      }
      std::get<I>(values) = *parsed;
      return ParseRowValues<I + 1>(row, values);
    }
  }

  template <size_t... I>
  void AppendRowValues(std::vector<std::string>& row,
                       const RowTuple& values,
                       std::index_sequence<I...>) const {
    (row.push_back(ntmap_detail::NetworkTableMapValueTraits<std::tuple_element_t<I, RowTuple>>::ToCsvString(
         std::get<I>(values))),
     ...);
  }

  template <size_t... I>
  RowTuple ReadRow(const RowTelemetry& row, std::index_sequence<I...>) const {
    return RowTuple{ntmap_detail::NetworkTableMapValueTraits<std::tuple_element_t<I, RowTuple>>::Read(
        std::get<I>(row).sub)...};
  }

  template <size_t... I>
  void WriteRow(RowTelemetry& row, const RowTuple& values, std::index_sequence<I...>) const {
    (ntmap_detail::NetworkTableMapValueTraits<std::tuple_element_t<I, RowTuple>>::Write(
         std::get<I>(row).pub, std::get<I>(values)),
     ...);
  }

  template <size_t I>
  void InitTelemetryCell(const std::shared_ptr<nt::NetworkTable>& keyTable,
                         const RowTuple& defaultValues,
                         RowTelemetry& row) const {
    using ValueType = std::tuple_element_t<I, RowTuple>;
    auto topic = ntmap_detail::NetworkTableMapValueTraits<ValueType>::GetTopic(keyTable, m_paramNames[I]);
    topic.SetRetained(true);
    topic.SetPersistent(true);

    auto& param = std::get<I>(row);
    param.pub = ntmap_detail::NetworkTableMapValueTraits<ValueType>::Publish(topic);
    param.sub = ntmap_detail::NetworkTableMapValueTraits<ValueType>::Subscribe(
        topic, std::get<I>(defaultValues));
  }

  template <size_t... I>
  void InitTelemetryCells(const std::shared_ptr<nt::NetworkTable>& keyTable,
                          const RowTuple& defaultValues,
                          RowTelemetry& row,
                          std::index_sequence<I...>) const {
    (InitTelemetryCell<I>(keyTable, defaultValues, row), ...);
  }

  /** Ensures publishers/subscribers exist for a row key. */
  void EnsureTelemetryForKey(const Key& key) const {
    if (m_telemetryMap.find(key) != m_telemetryMap.end()) {
      return;
    }

    auto keyTable = m_table->GetSubTable(NetworkTableMapKeyTraits<Key>::ToNetworkTableKey(key));
    auto row = std::make_unique<RowTelemetry>();

    RowTuple defaultValues{};
    auto valuesIt = m_localMap.find(key);
    if (valuesIt != m_localMap.end()) {
      defaultValues = valuesIt->second;
    } else {
      m_localMap[key] = RowTuple{};
    }

    InitTelemetryCells(keyTable, defaultValues, *row, std::index_sequence_for<RowTypes...>{});
    m_telemetryMap[key] = std::move(row);
  }

  /** Finds compatible NetworkTables subtables and creates telemetry for them. */
  void DiscoverNetworkTableRows() const {
    for (const auto& subkey : m_table->GetSubTables()) {
      auto key = NetworkTableMapKeyTraits<Key>::FromString(subkey);
      if (key.has_value()) {
        EnsureTelemetryForKey(*key);
      }
    }
  }

  std::shared_ptr<nt::NetworkTable> m_table;
  std::string m_keyColumnName;
  std::array<std::string, sizeof...(RowTypes)> m_paramNames;
  mutable MapType m_localMap;
  mutable std::map<Key, std::unique_ptr<RowTelemetry>> m_telemetryMap;
  std::string m_filePath;
};
