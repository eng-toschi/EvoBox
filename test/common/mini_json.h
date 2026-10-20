// Leitor JSON mínimo para os testes (só leitura do fixture). Header-only.
// Números guardam o texto original para conversão exata a inteiro.
#pragma once

#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace mj {

struct Value {
  enum class Type { Null, Bool, Number, String, Array, Object } type = Type::Null;
  bool b = false;
  std::string text;  // número (texto original) ou string
  std::vector<Value> arr;
  std::map<std::string, Value> obj;

  const Value& operator[](const std::string& k) const {
    static const Value null_value;
    auto it = obj.find(k);
    return it == obj.end() ? null_value : it->second;
  }
  bool has(const std::string& k) const { return obj.count(k) != 0; }
  double num() const { return std::strtod(text.c_str(), nullptr); }
  long long i64() const { return std::strtoll(text.c_str(), nullptr, 10); }
  const std::string& str() const { return text; }
};

class Parser {
 public:
  explicit Parser(const std::string& s) : s_(s) {}
  bool parse(Value* out) {
    ws();
    if (!value(out)) return false;
    ws();
    return i_ == s_.size();
  }

 private:
  void ws() {
    while (i_ < s_.size() && std::isspace(static_cast<unsigned char>(s_[i_]))) ++i_;
  }
  bool lit(const char* l) {
    const std::string w(l);
    if (s_.compare(i_, w.size(), w) != 0) return false;
    i_ += w.size();
    return true;
  }
  bool value(Value* v) {
    ws();
    if (i_ >= s_.size()) return false;
    const char c = s_[i_];
    if (c == '{') return object(v);
    if (c == '[') return array(v);
    if (c == '"') {
      v->type = Value::Type::String;
      return string(&v->text);
    }
    if (lit("true")) {
      v->type = Value::Type::Bool;
      v->b = true;
      return true;
    }
    if (lit("false")) {
      v->type = Value::Type::Bool;
      return true;
    }
    if (lit("null")) return true;
    return number(v);
  }
  bool number(Value* v) {
    const size_t start = i_;
    while (i_ < s_.size() && (std::isdigit(static_cast<unsigned char>(s_[i_])) ||
                              s_[i_] == '-' || s_[i_] == '+' || s_[i_] == '.' ||
                              s_[i_] == 'e' || s_[i_] == 'E'))
      ++i_;
    if (i_ == start) return false;
    v->type = Value::Type::Number;
    v->text = s_.substr(start, i_ - start);
    return true;
  }
  bool string(std::string* out) {
    if (s_[i_] != '"') return false;
    ++i_;
    while (i_ < s_.size() && s_[i_] != '"') {
      if (s_[i_] == '\\' && i_ + 1 < s_.size()) {
        ++i_;
        const char e = s_[i_];
        out->push_back(e == 'n' ? '\n' : e == 't' ? '\t' : e);
        if (e == 'u') i_ += 4;  // não usado no fixture; ignora o code point
      } else {
        out->push_back(s_[i_]);
      }
      ++i_;
    }
    if (i_ >= s_.size()) return false;
    ++i_;
    return true;
  }
  bool array(Value* v) {
    v->type = Value::Type::Array;
    ++i_;
    ws();
    if (i_ < s_.size() && s_[i_] == ']') {
      ++i_;
      return true;
    }
    for (;;) {
      Value e;
      if (!value(&e)) return false;
      v->arr.push_back(e);
      ws();
      if (i_ < s_.size() && s_[i_] == ',') {
        ++i_;
        continue;
      }
      if (i_ < s_.size() && s_[i_] == ']') {
        ++i_;
        return true;
      }
      return false;
    }
  }
  bool object(Value* v) {
    v->type = Value::Type::Object;
    ++i_;
    ws();
    if (i_ < s_.size() && s_[i_] == '}') {
      ++i_;
      return true;
    }
    for (;;) {
      ws();
      std::string k;
      if (i_ >= s_.size() || !string(&k)) return false;
      ws();
      if (i_ >= s_.size() || s_[i_] != ':') return false;
      ++i_;
      Value e;
      if (!value(&e)) return false;
      v->obj[k] = e;
      ws();
      if (i_ < s_.size() && s_[i_] == ',') {
        ++i_;
        continue;
      }
      if (i_ < s_.size() && s_[i_] == '}') {
        ++i_;
        return true;
      }
      return false;
    }
  }
  const std::string& s_;
  size_t i_ = 0;
};

inline bool load_file(const std::string& path, Value* out) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  std::stringstream ss;
  ss << f.rdbuf();
  const std::string s = ss.str();
  Parser p(s);
  return p.parse(out);
}

// Hex do fixture: ignora espaços (o fixture tem espaços dentro de alguns hex).
inline std::vector<uint8_t> hex(const std::string& h) {
  std::vector<uint8_t> out;
  int hi = -1;
  for (char c : h) {
    int v;
    if (c >= '0' && c <= '9') v = c - '0';
    else if (c >= 'a' && c <= 'f') v = c - 'a' + 10;
    else if (c >= 'A' && c <= 'F') v = c - 'A' + 10;
    else continue;
    if (hi < 0) {
      hi = v;
    } else {
      out.push_back(static_cast<uint8_t>(hi * 16 + v));
      hi = -1;
    }
  }
  return out;
}

}  // namespace mj
