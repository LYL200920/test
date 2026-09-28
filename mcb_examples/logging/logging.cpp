
#if defined (LOGGING_LOGGING_ENABLE)

#include <array>
#include <algorithm>

#include "logging.hpp"

char log_level::g_enabled_mask = error;

namespace
{
std::array<logging_hook*, 8> g_logging_hooks;
}

[[gnu::cold]] bool attach_logging_hook (logging_hook* h)
{
  if (std::find (g_logging_hooks.begin (), g_logging_hooks.end (), h)
      != g_logging_hooks.end ())
    return true;

  auto i = std::find (g_logging_hooks.begin (), g_logging_hooks.end (), nullptr);
  if (i == g_logging_hooks.end ())
    return false;

  *i = h;
  return true;
}

[[gnu::cold]] void detach_logging_hook (logging_hook* h)
{
  auto i = std::find (g_logging_hooks.begin (), g_logging_hooks.end (), h);
  if (i != g_logging_hooks.end ())
    *i = nullptr;
}

void logging_hooks_write (const void* data, size_t len)
{
  for (auto* h : g_logging_hooks)
    if (h != nullptr)
      h->logging_write (data, len);
}

#else // LOGGING_LOGGING_ENABLE

#include <cstdlib>

void logging_hooks_write (const void*, size_t) { }

#endif // LOGGING_LOGGING_ENABLE
