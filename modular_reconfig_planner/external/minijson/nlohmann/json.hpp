#pragma once

#include <cctype>
#include <cmath>
#include <istream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace nlohmann {

class json {
public:
    using object_t = std::map<std::string, json>;
    using array_t = std::vector<json>;
    using string_t = std::string;
    using boolean_t = bool;
    using number_t = double;

    json() : data_(nullptr) {}
    json(std::nullptr_t) : data_(nullptr) {}
    json(boolean_t v) : data_(v) {}
    json(number_t v) : data_(v) {}
    json(int v) : data_(static_cast<number_t>(v)) {}
    json(const char* v) : data_(string_t(v)) {}
    json(const string_t& v) : data_(v) {}
    json(string_t&& v) : data_(std::move(v)) {}
    json(const array_t& v) : data_(v) {}
    json(array_t&& v) : data_(std::move(v)) {}
    json(const object_t& v) : data_(v) {}
    json(object_t&& v) : data_(std::move(v)) {}

    static json array() { return json(array_t{}); }
    static json object() { return json(object_t{}); }

    bool is_null() const { return std::holds_alternative<std::nullptr_t>(data_); }
    bool is_boolean() const { return std::holds_alternative<boolean_t>(data_); }
    bool is_number() const { return std::holds_alternative<number_t>(data_); }
    bool is_string() const { return std::holds_alternative<string_t>(data_); }
    bool is_array() const { return std::holds_alternative<array_t>(data_); }
    bool is_object() const { return std::holds_alternative<object_t>(data_); }

    bool contains(const std::string& key) const {
        if (!is_object()) return false;
        const auto& obj = std::get<object_t>(data_);
        return obj.find(key) != obj.end();
    }

    const json& at(const std::string& key) const {
        if (!is_object()) throw std::out_of_range("json value is not an object");
        const auto& obj = std::get<object_t>(data_);
        auto it = obj.find(key);
        if (it == obj.end()) throw std::out_of_range("json object has no key: " + key);
        return it->second;
    }

    json& at(const std::string& key) {
        if (!is_object()) throw std::out_of_range("json value is not an object");
        auto& obj = std::get<object_t>(data_);
        auto it = obj.find(key);
        if (it == obj.end()) throw std::out_of_range("json object has no key: " + key);
        return it->second;
    }

    const json& at(std::size_t index) const {
        if (!is_array()) throw std::out_of_range("json value is not an array");
        const auto& arr = std::get<array_t>(data_);
        if (index >= arr.size()) throw std::out_of_range("json array index out of range");
        return arr[index];
    }

    json& at(std::size_t index) {
        if (!is_array()) throw std::out_of_range("json value is not an array");
        auto& arr = std::get<array_t>(data_);
        if (index >= arr.size()) throw std::out_of_range("json array index out of range");
        return arr[index];
    }

    std::size_t size() const {
        if (is_array()) return std::get<array_t>(data_).size();
        if (is_object()) return std::get<object_t>(data_).size();
        return 0;
    }

    array_t::iterator begin() {
        if (!is_array()) throw std::runtime_error("json value is not an array");
        return std::get<array_t>(data_).begin();
    }

    array_t::iterator end() {
        if (!is_array()) throw std::runtime_error("json value is not an array");
        return std::get<array_t>(data_).end();
    }

    array_t::const_iterator begin() const {
        if (!is_array()) throw std::runtime_error("json value is not an array");
        return std::get<array_t>(data_).begin();
    }

    array_t::const_iterator end() const {
        if (!is_array()) throw std::runtime_error("json value is not an array");
        return std::get<array_t>(data_).end();
    }

    template <typename T>
    T get() const {
        if constexpr (std::is_same_v<T, json>) {
            return *this;
        } else if constexpr (std::is_same_v<T, std::string>) {
            if (!is_string()) throw std::runtime_error("json value is not a string");
            return std::get<string_t>(data_);
        } else if constexpr (std::is_same_v<T, double>) {
            if (!is_number()) throw std::runtime_error("json value is not a number");
            return std::get<number_t>(data_);
        } else if constexpr (std::is_same_v<T, int>) {
            if (!is_number()) throw std::runtime_error("json value is not a number");
            return static_cast<int>(std::lround(std::get<number_t>(data_)));
        } else if constexpr (std::is_same_v<T, bool>) {
            if (!is_boolean()) throw std::runtime_error("json value is not a boolean");
            return std::get<boolean_t>(data_);
        } else if constexpr (std::is_same_v<T, std::vector<double>>) {
            if (!is_array()) throw std::runtime_error("json value is not an array of numbers");
            std::vector<double> out;
            for (const auto& item : std::get<array_t>(data_)) out.push_back(item.get<double>());
            return out;
        } else if constexpr (std::is_same_v<T, std::vector<std::string>>) {
            if (!is_array()) throw std::runtime_error("json value is not an array of strings");
            std::vector<std::string> out;
            for (const auto& item : std::get<array_t>(data_)) out.push_back(item.get<std::string>());
            return out;
        } else {
            static_assert(!sizeof(T), "Unsupported json::get<T>() type in lightweight JSON header");
        }
    }

    template <typename T>
    T value(const std::string& key, const T& defaultValue) const {
        if (!is_object()) return defaultValue;
        const auto& obj = std::get<object_t>(data_);
        auto it = obj.find(key);
        if (it == obj.end()) return defaultValue;
        return it->second.template get<T>();
    }

    template <typename T>
    T value(const char* key, const T& defaultValue) const {
        return value(std::string(key), defaultValue);
    }

    std::string dump() const {
        if (is_null()) return "null";
        if (is_boolean()) return std::get<boolean_t>(data_) ? "true" : "false";
        if (is_number()) return std::to_string(std::get<number_t>(data_));
        if (is_string()) return '"' + std::get<string_t>(data_) + '"';
        if (is_array()) {
            std::string s = "[";
            const auto& arr = std::get<array_t>(data_);
            for (std::size_t i = 0; i < arr.size(); ++i) {
                if (i) s += ",";
                s += arr[i].dump();
            }
            return s + "]";
        }
        std::string s = "{";
        const auto& obj = std::get<object_t>(data_);
        std::size_t i = 0;
        for (const auto& [k, v] : obj) {
            if (i++) s += ",";
            s += "\"" + k + "\":" + v.dump();
        }
        return s + "}";
    }

private:
    class parser {
    public:
        explicit parser(std::string text) : text_(std::move(text)) {}

        json parse() {
            skip_ws();
            json v = parse_value();
            skip_ws();
            if (pos_ != text_.size()) throw std::runtime_error("Unexpected trailing characters in JSON");
            return v;
        }

    private:
        void skip_ws() {
            while (pos_ < text_.size() && (text_[pos_] == ' ' || text_[pos_] == '\n' || text_[pos_] == '\r' || text_[pos_] == '\t')) ++pos_;
        }

        char peek() const { return pos_ < text_.size() ? text_[pos_] : '\0'; }

        char getch() {
            if (pos_ >= text_.size()) throw std::runtime_error("Unexpected end of JSON");
            return text_[pos_++];
        }

        void expect(char c) {
            skip_ws();
            if (getch() != c) throw std::runtime_error(std::string("Expected '") + c + "' in JSON");
        }

        json parse_value() {
            skip_ws();
            char c = peek();
            if (c == '{') return parse_object();
            if (c == '[') return parse_array();
            if (c == '"') return json(parse_string());
            if (c == '-' || std::isdigit(static_cast<unsigned char>(c))) return json(parse_number());
            if (match("true")) return json(true);
            if (match("false")) return json(false);
            if (match("null")) return json(nullptr);
            throw std::runtime_error("Invalid JSON value at offset " + std::to_string(pos_));
        }

        bool match(const std::string& token) {
            if (text_.compare(pos_, token.size(), token) == 0) {
                pos_ += token.size();
                return true;
            }
            return false;
        }

        json parse_object() {
            expect('{');
            object_t obj;
            skip_ws();
            if (peek() == '}') { ++pos_; return json(std::move(obj)); }
            while (true) {
                skip_ws();
                if (peek() != '"') throw std::runtime_error("Expected object key string in JSON");
                std::string key = parse_string();
                expect(':');
                obj[key] = parse_value();
                skip_ws();
                char c = getch();
                if (c == '}') break;
                if (c != ',') throw std::runtime_error(std::string("Expected , or } in JSON object at offset ") + std::to_string(pos_-1) + ", got [" + c + "]");
            }
            return json(std::move(obj));
        }

        json parse_array() {
            expect('[');
            array_t arr;
            skip_ws();
            if (peek() == ']') { ++pos_; return json(std::move(arr)); }
            while (true) {
                arr.push_back(parse_value());
                skip_ws();
                char c = getch();
                if (c == ']') break;
                if (c != ',') throw std::runtime_error(std::string("Expected , or ] in JSON array at offset ") + std::to_string(pos_-1) + ", got [" + c + "]");
            }
            return json(std::move(arr));
        }

        std::string parse_string() {
            expect('"');
            std::string s;
            while (true) {
                char c = getch();
                if (c == '"') break;
                if (c == '\\') {
                    char e = getch();
                    switch (e) {
                        case '"': s.push_back('"'); break;
                        case '\\': s.push_back('\\'); break;
                        case '/': s.push_back('/'); break;
                        case 'b': s.push_back('\b'); break;
                        case 'f': s.push_back('\f'); break;
                        case 'n': s.push_back('\n'); break;
                        case 'r': s.push_back('\r'); break;
                        case 't': s.push_back('\t'); break;
                        case 'u': {
                            // Lightweight fallback: preserve non-ASCII escapes as '?'.
                            for (int i = 0; i < 4; ++i) getch();
                            s.push_back('?');
                            break;
                        }
                        default: throw std::runtime_error("Unsupported JSON string escape");
                    }
                } else {
                    s.push_back(c);
                }
            }
            return s;
        }

        double parse_number() {
            std::size_t start = pos_;
            if (peek() == '-') ++pos_;
            while (std::isdigit(static_cast<unsigned char>(peek()))) ++pos_;
            if (peek() == '.') {
                ++pos_;
                while (std::isdigit(static_cast<unsigned char>(peek()))) ++pos_;
            }
            if (peek() == 'e' || peek() == 'E') {
                ++pos_;
                if (peek() == '+' || peek() == '-') ++pos_;
                while (std::isdigit(static_cast<unsigned char>(peek()))) ++pos_;
            }
            return std::stod(text_.substr(start, pos_ - start));
        }

        std::string text_;
        std::size_t pos_ = 0;
    };

    std::variant<std::nullptr_t, boolean_t, number_t, string_t, array_t, object_t> data_;

    friend std::istream& operator>>(std::istream& is, json& j);
};

inline std::istream& operator>>(std::istream& is, json& j) {
    std::ostringstream ss;
    ss << is.rdbuf();
    json::parser p(ss.str());
    j = p.parse();
    return is;
}

} // namespace nlohmann
