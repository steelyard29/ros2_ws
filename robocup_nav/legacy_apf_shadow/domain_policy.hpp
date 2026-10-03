#pragma once
#include <string>

namespace apf_domain {
inline bool entry_allowed(const char *domain) {
  if (!domain) return false;
  const std::string value(domain);
  return value == "176" || value == "0" || value == "186";
}
inline bool configured_allowed(const char *domain, bool real, bool isolated) {
  if (!entry_allowed(domain)) return false;
  const std::string value(domain);
  // 'real' also enables strict path freshness in synthetic production-chain tests.
  // It does not select a domain or connect hardware by itself.
  if (isolated) return value == "186";
  return value == "176" || (value == "0" && real);
}
}
