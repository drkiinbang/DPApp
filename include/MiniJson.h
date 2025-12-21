#pragma once
#include <unordered_map>
#include <variant>
#include <optional>
#include <cctype>
#include <string>

namespace DPApp {
	using JsonValue = std::variant<std::string, double, bool>;
	using JsonObject = std::unordered_map<std::string, JsonValue>;

	const size_t MAX_DEPTH = 32;
	class MiniJson {
	public:
		MiniJson() {}

		/// ê³µë°±/ê°œí–‰ ìŠ¤í‚µ
		static void skip_ws(const std::string& s, size_t& i) {
			while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
		}

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
					default: return false; // \uXXXX ë¯¸ì§€ì›(ê°„ì´)
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

		static bool parse_number(const std::string& s, size_t& i, double& out) {
			size_t start = i;
			if (i < s.size() && (s[i] == '-' || s[i] == '+')) ++i;
			bool has_digit = false;
			while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) { ++i; has_digit = true; }
			if (i < s.size() && s[i] == '.') {
				++i;
				while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) { ++i; has_digit = true; }
			}
			if (!has_digit) return false;
			try { out = std::stod(s.substr(start, i - start)); }
			catch (...) { return false; }
			return true;
		}

		static bool parse_bool(const std::string& s, size_t& i, bool& out) {
			if (s.compare(i, 4, "true") == 0) { i += 4; out = true; return true; }
			if (s.compare(i, 5, "false") == 0) { i += 5; out = false; return true; }
			return false;
		}

		/// MAX_DEPTH, 32, ,ì´ˆê³¼í•˜ë©´ failure
		static std::optional<JsonObject> parse_object(const std::string& s, size_t depth = 0) {
			if (depth > MAX_DEPTH) {
				return std::nullopt;
			}

			size_t i = 0;
			skip_ws(s, i);     // ê³µë°±/ê°œí–‰ ìŠ¤í‚µ
			if (i >= s.size() || s[i] != '{')
				return std::nullopt;
			++i;               // '{' ì†Œë¹„

			JsonObject obj;

			for (;;) {
				skip_ws(s, i);
				/// ë¹ˆ ê°ì²´ {} ì²˜ë¦¬: ë‹«ëŠ" ê´„í˜¸ '}'ë©´ ì¢…ë£Œ
				if (i < s.size() && s[i] == '}') {
					++i;
					break;
				}

				// í‚¤ëŠ" ë°˜ë"œì‹œ ë¬¸ìžì—´
				std::string key;
				if (!parse_string(s, i, key))
					return std::nullopt;

				skip_ws(s, i);
				if (i >= s.size() || s[i] != ':')
					return std::nullopt;
				++i;                 // ':' ì†Œë¹„
				skip_ws(s, i);

				if (i >= s.size())
					return std::nullopt;

				// [í•µì‹¬ #2] í˜„ ë²„ì „ì€ ê°'ìœ¼ë¡œ "ë¬¸ìžì—´/ìˆ«ìž/ë¶ˆë¦¬ì–¸"ë§Œ í—ˆìš©
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
					/// ìž¬ê·€ í˜¸ì¶œ(parse_object(..., depth+1))ì„ í•˜ì§€ ì•Šìœ¼ë¯€ë¡œ
					/// ì‹¤ì œë¡œëŠ" depthê°€ 0ì—ì„œ ì¦ê°€í•  ì¼ì´ ì—†ìŒ
					return std::nullopt;
				}
				else {
					bool b;
					if (!parse_bool(s, i, b))
						return std::nullopt;
					obj.emplace(std::move(key), JsonValue{ b });
				}

				skip_ws(s, i);
				if (i < s.size() && s[i] == ',') { ++i; continue; } // ë‹¤ìŒ KVìŒ
				if (i < s.size() && s[i] == '}') { ++i; break; }    // ê°ì²´ ì¢…ë£Œ
				if (i >= s.size()) return std::nullopt;             // ìž˜ëª»ëœ ì¢…ë£Œ
			}
			return obj;
		}

		template<typename T>
		static std::optional<T> get(const JsonObject& o, const std::string& k) {
			auto it = o.find(k);
			if (it == o.end()) return std::nullopt;
			if (auto p = std::get_if<T>(&it->second)) return *p;
			return std::nullopt;
		}
	};
}