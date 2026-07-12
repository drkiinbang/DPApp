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

    /// 재귀 타입을 위한 전방 선언
    struct JsonValueEx;

    /// JSON 객체 타입 (문자열 -> JsonValueEx)
    using JsonObjectEx = std::unordered_map<std::string, JsonValueEx>;

    /// JSON 배열 타입 (JsonValueEx의 vector)
    using JsonArrayEx = std::vector<JsonValueEx>;

    /// 확장 JSON 값 - 문자열, 실수, 불리언, null, 객체, 배열을 지원
    struct JsonValueEx {
        std::variant<
            std::nullptr_t,                         /// null
            std::string,                            /// 문자열
            double,                                 /// 숫자
            bool,                                   /// 불리언
            std::shared_ptr<JsonObjectEx>,          /// 중첩 객체
            std::shared_ptr<JsonArrayEx>            /// 배열
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
    /// 하위 호환을 위한 레거시 타입
    /// =========================================
    using JsonValue = std::variant<std::string, double, bool>;
    using JsonObject = std::unordered_map<std::string, JsonValue>;

    /// =========================================
    /// MiniJson 파서
    /// =========================================
    class MiniJson {
    public:
        MiniJson() {}

        /// 공백 건너뛰기
        static void skip_ws(const std::string& s, size_t& i) {
            while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
        }

        /// 문자열 파싱
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
                        /// \uXXXX 건너뛰기 (단순화 -- 16진수 4자리만 건너뜀)
                        if (i + 4 <= s.size()) {
                            i += 4;
                            tmp.push_back('?');  /// 자리표시자(placeholder)
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

        /// 숫자 파싱
        static bool parse_number(const std::string& s, size_t& i, double& out) {
            size_t start = i;
            if (i < s.size() && (s[i] == '-' || s[i] == '+')) ++i;
            bool has_digit = false;
            while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) { ++i; has_digit = true; }
            if (i < s.size() && s[i] == '.') {
                ++i;
                while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) { ++i; has_digit = true; }
            }
            /// 지수 표기 처리 (예: 1e10, 2.5E-3)
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

        /// 불리언 파싱
        static bool parse_bool(const std::string& s, size_t& i, bool& out) {
            if (s.compare(i, 4, "true") == 0) { i += 4; out = true; return true; }
            if (s.compare(i, 5, "false") == 0) { i += 5; out = false; return true; }
            return false;
        }

        /// null 파싱
        static bool parse_null(const std::string& s, size_t& i) {
            if (s.compare(i, 4, "null") == 0) { i += 4; return true; }
            return false;
        }

        /// =========================================
        /// 확장 파서 - 배열과 중첩 객체를 지원
        /// =========================================

        /// 임의의 JSON 값 파싱 (재귀)
        static bool parse_value_ex(const std::string& s, size_t& i, JsonValueEx& out, size_t depth = 0) {
            if (depth > MAX_DEPTH) return false;

            skip_ws(s, i);
            if (i >= s.size()) return false;

            char c = s[i];

            /// 문자열
            if (c == '"') {
                std::string str;
                if (!parse_string(s, i, str)) return false;
                out = JsonValueEx(std::move(str));
                return true;
            }
            /// 숫자
            else if (std::isdigit(static_cast<unsigned char>(c)) || c == '-' || c == '+') {
                double d;
                if (!parse_number(s, i, d)) return false;
                out = JsonValueEx(d);
                return true;
            }
            /// 객체
            else if (c == '{') {
                auto obj = std::make_shared<JsonObjectEx>();
                if (!parse_object_ex_internal(s, i, *obj, depth + 1)) return false;
                out = JsonValueEx(obj);
                return true;
            }
            /// 배열
            else if (c == '[') {
                auto arr = std::make_shared<JsonArrayEx>();
                if (!parse_array_ex_internal(s, i, *arr, depth + 1)) return false;
                out = JsonValueEx(arr);
                return true;
            }
            /// 불리언 또는 null
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

        /// 객체 파싱 (내부용, 인덱스 매개변수 사용)
        static bool parse_object_ex_internal(const std::string& s, size_t& i, JsonObjectEx& obj, size_t depth = 0) {
            if (depth > MAX_DEPTH) return false;

            skip_ws(s, i);
            if (i >= s.size() || s[i] != '{') return false;
            ++i;  /// '{' 소비

            for (;;) {
                skip_ws(s, i);

                /// 빈 객체 또는 끝
                if (i < s.size() && s[i] == '}') {
                    ++i;
                    return true;
                }

                /// 키 파싱
                std::string key;
                if (!parse_string(s, i, key)) return false;

                skip_ws(s, i);
                if (i >= s.size() || s[i] != ':') return false;
                ++i;  /// ':' 소비

                /// 값 파싱
                JsonValueEx value;
                if (!parse_value_ex(s, i, value, depth)) return false;

                obj.emplace(std::move(key), std::move(value));

                skip_ws(s, i);
                if (i < s.size() && s[i] == ',') { ++i; continue; }
                if (i < s.size() && s[i] == '}') { ++i; return true; }
                return false;
            }
        }

        /// 배열 파싱 (내부용)
        static bool parse_array_ex_internal(const std::string& s, size_t& i, JsonArrayEx& arr, size_t depth = 0) {
            if (depth > MAX_DEPTH) return false;

            skip_ws(s, i);
            if (i >= s.size() || s[i] != '[') return false;
            ++i;  /// '[' 소비

            for (;;) {
                skip_ws(s, i);

                /// 빈 배열 또는 끝
                if (i < s.size() && s[i] == ']') {
                    ++i;
                    return true;
                }

                /// 값 파싱
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
        /// 공개 확장 API
        /// =========================================

        /// JSON 문자열을 확장 객체로 파싱
        static std::optional<JsonObjectEx> parse_object_ex(const std::string& s) {
            size_t i = 0;
            JsonObjectEx obj;
            if (parse_object_ex_internal(s, i, obj, 0)) {
                return obj;
            }
            return std::nullopt;
        }

        /// JSON 문자열을 확장 배열로 파싱
        static std::optional<JsonArrayEx> parse_array_ex(const std::string& s) {
            size_t i = 0;
            JsonArrayEx arr;
            if (parse_array_ex_internal(s, i, arr, 0)) {
                return arr;
            }
            return std::nullopt;
        }

        /// 확장 객체에서 값 조회
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

        /// 확장 객체에서 배열 조회
        static std::optional<JsonArrayEx> get_array(const JsonObjectEx& o, const std::string& k) {
            auto it = o.find(k);
            if (it == o.end()) return std::nullopt;
            if (it->second.is_array()) return it->second.as_array();
            return std::nullopt;
        }

        /// 확장 객체에서 중첩 객체 조회
        static std::optional<JsonObjectEx> get_object(const JsonObjectEx& o, const std::string& k) {
            auto it = o.find(k);
            if (it == o.end()) return std::nullopt;
            if (it->second.is_object()) return it->second.as_object();
            return std::nullopt;
        }

        /// =========================================
        /// 레거시 API (하위 호환)
        /// =========================================

        /// 단순 객체 파싱 (배열 없음, 중첩 객체 없음)
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
                    /// 레거시 모드에서는 중첩 객체를 지원하지 않음
                    return std::nullopt;
                }
                else if (s[i] == '[') {
                    /// 레거시 모드에서는 배열을 지원하지 않음
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

        /// 레거시 객체에서 값 조회
        template<typename T>
        static std::optional<T> get(const JsonObject& o, const std::string& k) {
            auto it = o.find(k);
            if (it == o.end()) return std::nullopt;
            if (auto p = std::get_if<T>(&it->second)) return *p;
            return std::nullopt;
        }
    };

}
