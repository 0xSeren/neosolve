//-----------------------------------------------------------------------------
// Profiling infrastructure for performance measurement.
//-----------------------------------------------------------------------------
#ifndef SOLVESPACE_PROFILER_H
#define SOLVESPACE_PROFILER_H

#include <string>
#include <vector>
#include <cstdint>

namespace SolveSpace {

struct ProfileEntry {
    const char *name;
    uint64_t    startMs;
    uint64_t    durationMs;
    int         depth;
};

class Profiler {
public:
    static void Begin(const char *name);
    static void End();
    static void Reset();
    static void Report();
    static void ReportToFile(const std::string &path);
    static bool IsEnabled();
    static void SetEnabled(bool enabled);

    static std::vector<ProfileEntry> &GetEntries();

private:
    static bool enabled;
    static int currentDepth;
    static std::vector<ProfileEntry> entries;
    static std::vector<uint64_t> startStack;
};

class ProfileScope {
public:
    ProfileScope(const char *name) : name(name) {
        if(Profiler::IsEnabled()) Profiler::Begin(name);
    }
    ~ProfileScope() {
        if(Profiler::IsEnabled()) Profiler::End();
    }
private:
    const char *name;
};

} // namespace SolveSpace

#ifdef ENABLE_PROFILING
    #define PROFILE_SCOPE(name) SolveSpace::ProfileScope _profileScope##__LINE__(name)
    #define PROFILE_FUNCTION() SolveSpace::ProfileScope _profileScope##__LINE__(__func__)
#else
    #define PROFILE_SCOPE(name) ((void)0)
    #define PROFILE_FUNCTION() ((void)0)
#endif

#endif // SOLVESPACE_PROFILER_H
