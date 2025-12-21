#pragma once
#include <unordered_map>
#include <variant>
#include <optional>
#include <vector>
#include <cctype>
#include <string>
#include <memory>

namespace DPApp {

    const size_t MAX_DEPTH = 32;

    /// Forward declaration for recursive types
    struct JsonValueEx;

    /// JSON Object type (string -> JsonValueEx)
    using JsonObjectEx = std::unordered_map<std::string, JsonValueEx>;

    /// JSON Array type (vector of JsonValueEx)
    using JsonArrayEx = std::vector<JsonValueEx>;

    /// Extended JSON Value - supports string, double, bool, null, object, array
    struct JsonValueEx {
        std::variant<
            std::nullptr_t,                         /// null
            std::string,                            /// string
            double,                                 /// number
            bool,                                   /// boolean
            std::shared_ptr<JsonObjectEx>,          /// nested object
            std::shared_ptr<JsonArrayEx>            /// array
        > data;

        JsonValueEx() : data(nullptr) {}
        JsonValueEx(std::nullptr_t) : data(nullptr) {}
        JsonValueEx(const std::string& s) : data(s) {}
        JsonValueEx(std::string&& s) : data(std::move(s)) {}
        JsonValueEx(const char* s) : data(std::string(s)) {}
        JsonValueEx(double d) : data(d) {}
        JsonValueEx(bool b) : data(b) {}
        JsonValueEx(std::shared_ptr<JsonObjectEx> obj) : data(obj) {}
        JsonValueEx(std::shared_ptr<JsonArrayEx> arr) : data(arr) {}

        bool is_null() const { return std::holds_alternative<std::nullptr_t>(data); }
        bool is_string() const { return std::holds_alternative<std::string>(data); }
        bool is_number() const { return std::holds_alternative<double>(data); }
        bool is_bool() const { return std::holds_alternative<bool>(data); }
        bool is_object() const { return std::holds_alternative<std::shared_ptr<JsonObjectEx>>(data); }
        bool is_array() const { return std::holds_alternative<std::shared_ptr<JsonArrayEx>>(data); }

        std::string as_string() const { return std::get<std::string>(data); }
        double as_number() const { return std::get<double>(data); }
        bool as_bool() const { return std::get<bool>(data); }
        JsonObjectEx& as_object() const { return *std::get<std::shared_ptr<JsonObjectEx>>(data); }
        JsonArrayEx& as_array() const { return *std::get<std::shared_ptr<JsonArrayEx>>(data); }
    };

    /// =========================================
    /// Legacy types for backward compatibility
    /// =========================================
    using JsonValue = std::variant<std::string, double, bool>;
    using JsonObject = std::unordered_map<std::string, JsonValue>;

    /// =========================================
    /// MiniJson Parser
    /// =========================================
    class MiniJson {
    public:
        MiniJson() {}

        /// Skip whitespace
        static void skip_ws(const std::string& s, size_t& i) {
            while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
        }

        /// Parse string
        static bool parse_string(const std::string& s, size_t& i, std::string& out, size_t max_length = 1024 * 1024) {
            if (i >= s.size() || s[i] != '"') return false;
            ++i;
            std::string tmp;
            tmp.reserve(512);

            while (i < s.size()) {
                if (tmp.size() > max_length) {
                    return false;
                }

                char c = s[i++];
                if (c == '\\') {
                    if (i >= s.size()) return false;
                    char e = s[i++];
                    switch (e) {
                    case '"': tmp.push_back('"'); break;
                    case '\\': tmp.push_back('\\'); break;
                    case '/': tmp.push_back('/'); break;
                    case 'b': tmp.push_back('\b'); break;
                    case 'f': tmp.push_back('\f'); break;
                    case 'n': tmp.push_back('\n'); break;
                    case 'r': tmp.push_back('\r'); break;
                    case 't': tmp.push_back('\t'); break;
                    case 'u': {
                        /// Skip \uXXXX (simplified - just skip 4 hex digits)
                        if (i + 4 <= s.size()) {
                            i += 4;
                            tmp.push_back('?');  /// Placeholder
                        }
                        else return false;
                        break;
                    }
                    default: return false;
                    }
                }
                else if (c == '"') {
                    out.swap(tmp); return true;
                }
                else {
                    tmp.push_back(c);
                }
            }
            return false;
        }

        /// Parse number
        static bool parse_number(const std::string& s, size_t& i, double& out) {
            size_t start = i;
            if (i < s.size() && (s[i] == '-' || s[i] == '+')) ++i;
            bool has_digit = false;
            while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) { ++i; has_digit = true; }
            if (i < s.size() && s[i] == '.') {
                ++i;
                while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) { ++i; has_digit = true; }
            }
            /// Handle exponent (e.g., 1e10, 2.5E-3)
            if (i < s.size() && (s[i] == 'e' || s[i] == 'E')) {
                ++i;
                if (i < s.size() && (s[i] == '+' || s[i] == '-')) ++i;
                while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) ++i;
            }
            if (!has_digit) return false;
            try { out = std::stod(s.substr(start, i - start)); }
            catch (...) { return false; }
            return true;
        }

        /// Parse boolean
        static bool parse_bool(const std::string& s, size_t& i, bool& out) {
            if (s.compare(i, 4, "true") == 0) { i += 4; out = true; return true; }
            if (s.compare(i, 5, "false") == 0) { i += 5; out = false; return true; }
            return false;
        }

        /// Parse null
        static bool parse_null(const std::string& s, size_t& i) {
            if (s.compare(i, 4, "null") == 0) { i += 4; return true; }
            return false;
        }

        /// =========================================
        /// Extended Parser - supports arrays and nested objects
        /// =========================================

        /// Parse any JSON value (recursive)
        static bool parse_value_ex(const std::string& s, size_t& i, JsonValueEx& out, size_t depth = 0) {
            if (depth > MAX_DEPTH) return false;

            skip_ws(s, i);
            if (i >= s.size()) return false;

            char c = s[i];

            /// String
            if (c == '"') {
                std::string str;
                if (!parse_string(s, i, str)) return false;
                out = JsonValueEx(std::move(str));
                return true;
            }
            /// Number
            else if (std::isdigit(static_cast<unsigned char>(c)) || c == '-' || c == '+') {
                double d;
                if (!parse_number(s, i, d)) return false;
                out = JsonValueEx(d);
                return true;
            }
            /// Object
            else if (c == '{') {
                auto obj = std::make_shared<JsonObjectEx>();
                if (!parse_object_ex_internal(s, i, *obj, depth + 1)) return false;
                out = JsonValueEx(obj);
                return true;
            }
            /// Array
            else if (c == '[') {
                auto arr = std::make_shared<JsonArrayEx>();
                if (!parse_array_ex_internal(s, i, *arr, depth + 1)) return false;
                out = JsonValueEx(arr);
                return true;
            }
            /// Boolean or null
            else if (c == 't' || c == 'f') {
                bool b;
                if (!parse_bool(s, i, b)) return false;
                out = JsonValueEx(b);
                return true;
            }
            else if (c == 'n') {
                if (!parse_null(s, i)) return false;
                out = JsonValueEx(nullptr);
                return true;
            }

            return false;
        }

        /// Parse object (internal, with index parameter)
        static bool parse_object_ex_internal(const std::string& s, size_t& i, JsonObjectEx& obj, size_t depth = 0) {
            if (depth > MAX_DEPTH) return false;

            skip_ws(s, i);
            if (i >= s.size() || s[i] != '{') return false;
            ++i;  /// consume '{'

            for (;;) {
                skip_ws(s, i);

                /// Empty object or end
                if (i < s.size() && s[i] == '}') {
                    ++i;
                    return true;
                }

                /// Parse key
                std::string key;
                if (!parse_string(s, i, key)) return false;

                skip_ws(s, i);
                if (i >= s.size() || s[i] != ':') return false;
                ++i;  /// consume ':'

                /// Parse value
                JsonValueEx value;
                if (!parse_value_ex(s, i, value, depth)) return false;

                obj.emplace(std::move(key), std::move(value));

                skip_ws(s, i);
                if (i < s.size() && s[i] == ',') { ++i; continue; }
                if (i < s.size() && s[i] == '}') { ++i; return true; }
                return false;
            }
        }

        /// Parse array (internal)
        static bool parse_array_ex_internal(const std::string& s, size_t& i, JsonArrayEx& arr, size_t depth = 0) {
            if (depth > MAX_DEPTH) return false;

            skip_ws(s, i);
            if (i >= s.size() || s[i] != '[') return false;
            ++i;  /// consume '['

            for (;;) {
                skip_ws(s, i);

                /// Empty array or end
                if (i < s.size() && s[i] == ']') {
                    ++i;
                    return true;
                }

                /// Parse value
                JsonValueEx value;
                if (!parse_value_ex(s, i, value, depth)) return false;

                arr.push_back(std::move(value));

                skip_ws(s, i);
                if (i < s.size() && s[i] == ',') { ++i; continue; }
                if (i < s.size() && s[i] == ']') { ++i; return true; }
                return false;
            }
        }

        /// =========================================
        /// Public Extended API
        /// =========================================

        /// Parse JSON string to extended object
        static std::optional<JsonObjectEx> parse_object_ex(const std::string& s) {
            size_t i = 0;
            JsonObjectEx obj;
            if (parse_object_ex_internal(s, i, obj, 0)) {
                return obj;
            }
            return std::nullopt;
        }

        /// Parse JSON string to extended array
        static std::optional<JsonArrayEx> parse_array_ex(const std::string& s) {
            size_t i = 0;
            JsonArrayEx arr;
            if (parse_array_ex_internal(s, i, arr, 0)) {
                return arr;
            }
            return std::nullopt;
        }

        /// Get value from extended object
        template<typename T>
        static std::optional<T> get_ex(const JsonObjectEx& o, const std::string& k) {
            auto it = o.find(k);
            if (it == o.end()) return std::nullopt;

            const JsonValueEx& val = it->second;

            if constexpr (std::is_same_v<T, std::string>) {
                if (val.is_string()) return val.as_string();
            }
            else if constexpr (std::is_same_v<T, double>) {
                if (val.is_number()) return val.as_number();
            }
            else if constexpr (std::is_same_v<T, bool>) {
                if (val.is_bool()) return val.as_bool();
            }
            else if constexpr (std::is_same_v<T, JsonObjectEx>) {
                if (val.is_object()) return val.as_object();
            }
            else if constexpr (std::is_same_v<T, JsonArrayEx>) {
                if (val.is_array()) return val.as_array();
            }

            return std::nullopt;
        }

        /// Get array from extended object
        static std::optional<JsonArrayEx> get_array(const JsonObjectEx& o, const std::string& k) {
            auto it = o.find(k);
            if (it == o.end()) return std::nullopt;
            if (it->second.is_array()) return it->second.as_array();
            return std::nullopt;
        }

        /// Get nested object from extended object
        static std::optional<JsonObjectEx> get_object(const JsonObjectEx& o, const std::string& k) {
            auto it = o.find(k);
            if (it == o.end()) return std::nullopt;
            if (it->second.is_object()) return it->second.as_object();
            return std::nullopt;
        }

        /// =========================================
        /// Legacy API (backward compatible)
        /// =========================================

        /// Parse simple object (no arrays, no nested objects)
        static std::optional<JsonObject> parse_object(const std::string& s, size_t depth = 0) {
            if (depth > MAX_DEPTH) {
                return std::nullopt;
            }

            size_t i = 0;
            skip_ws(s, i);
            if (i >= s.size() || s[i] != '{')
                return std::nullopt;
            ++i;

            JsonObject obj;

            for (;;) {
                skip_ws(s, i);
                if (i < s.size() && s[i] == '}') {
                    ++i;
                    break;
                }

                std::string key;
                if (!parse_string(s, i, key))
                    return std::nullopt;

                skip_ws(s, i);
                if (i >= s.size() || s[i] != ':')
                    return std::nullopt;
                ++i;
                skip_ws(s, i);

                if (i >= s.size())
                    return std::nullopt;

                if (s[i] == '"') {
                    std::string v;
                    if (!parse_string(s, i, v))
                        return std::nullopt;
                    obj.emplace(std::move(key), JsonValue{ std::move(v) });
                }
                else if (std::isdigit(static_cast<unsigned char>(s[i])) || s[i] == '-' || s[i] == '+') {
                    double d;
                    if (!parse_number(s, i, d))
                        return std::nullopt;
                    obj.emplace(std::move(key), JsonValue{ d });
                }
                else if (s[i] == '{') {
                    /// Skip nested objects in legacy mode
                    return std::nullopt;
                }
                else if (s[i] == '[') {
                    /// Skip arrays in legacy mode
                    return std::nullopt;
                }
                else {
                    bool b;
                    if (!parse_bool(s, i, b))
                        return std::nullopt;
                    obj.emplace(std::move(key), JsonValue{ b });
                }

                skip_ws(s, i);
                if (i < s.size() && s[i] == ',') { ++i; continue; }
                if (i < s.size() && s[i] == '}') { ++i; break; }
                if (i >= s.size()) return std::nullopt;
            }
            return obj;
        }

        /// Get value from legacy object
        template<typename T>
        static std::optional<T> get(const JsonObject& o, const std::string& k) {
            auto it = o.find(k);
            if (it == o.end()) return std::nullopt;
            if (auto p = std::get_if<T>(&it->second)) return *p;
            return std::nullopt;
        }
    };

}