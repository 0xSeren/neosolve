//-----------------------------------------------------------------------------
// Profiling infrastructure for performance measurement.
//-----------------------------------------------------------------------------
#include "profiler.h"
#include "util.h"
#include <cstdio>
#include <cstring>
#include <algorithm>

namespace SolveSpace {

bool Profiler::enabled = false;
int Profiler::currentDepth = 0;
std::vector<ProfileEntry> Profiler::entries;
std::vector<uint64_t> Profiler::startStack;

void Profiler::Begin(const char *name) {
    if(!enabled) return;

    uint64_t now = GetMilliseconds();
    startStack.push_back(now);

    ProfileEntry entry = {};
    entry.name = name;
    entry.startMs = now;
    entry.depth = currentDepth;
    entries.push_back(entry);

    currentDepth++;
}

void Profiler::End() {
    if(!enabled || startStack.empty()) return;

    uint64_t now = GetMilliseconds();
    uint64_t start = startStack.back();
    startStack.pop_back();
    currentDepth--;

    // Find the matching entry (last one at current depth)
    for(int i = (int)entries.size() - 1; i >= 0; i--) {
        if(entries[i].depth == currentDepth && entries[i].durationMs == 0) {
            entries[i].durationMs = now - start;
            break;
        }
    }
}

void Profiler::Reset() {
    entries.clear();
    startStack.clear();
    currentDepth = 0;
}

bool Profiler::IsEnabled() {
    return enabled;
}

void Profiler::SetEnabled(bool e) {
    enabled = e;
    if(enabled) {
        Reset();
    }
}

std::vector<ProfileEntry> &Profiler::GetEntries() {
    return entries;
}

void Profiler::Report() {
    if(entries.empty()) {
        fprintf(stdout, "No profiling data collected.\n");
        return;
    }

    fprintf(stdout, "\n=== Profile Report ===\n");

    for(const auto &e : entries) {
        for(int i = 0; i < e.depth; i++) {
            fprintf(stdout, "  ");
        }
        fprintf(stdout, "%s: %llu ms\n", e.name, (unsigned long long)e.durationMs);
    }

    // Summary: aggregate by name
    fprintf(stdout, "\n=== Summary ===\n");

    struct Summary {
        const char *name;
        uint64_t totalMs;
        int count;
    };
    std::vector<Summary> summaries;

    for(const auto &e : entries) {
        bool found = false;
        for(auto &s : summaries) {
            if(strcmp(s.name, e.name) == 0) {
                s.totalMs += e.durationMs;
                s.count++;
                found = true;
                break;
            }
        }
        if(!found) {
            Summary s = { e.name, e.durationMs, 1 };
            summaries.push_back(s);
        }
    }

    // Sort by total time descending
    std::sort(summaries.begin(), summaries.end(),
        [](const Summary &a, const Summary &b) {
            return a.totalMs > b.totalMs;
        });

    for(const auto &s : summaries) {
        fprintf(stdout, "%-30s %8llu ms (%d calls, avg %.1f ms)\n",
            s.name,
            (unsigned long long)s.totalMs,
            s.count,
            s.count > 0 ? (double)s.totalMs / s.count : 0.0);
    }
}

void Profiler::ReportToFile(const std::string &path) {
    FILE *f = fopen(path.c_str(), "w");
    if(!f) {
        fprintf(stderr, "Could not open %s for writing\n", path.c_str());
        return;
    }

    // JSON output
    fprintf(f, "{\n  \"entries\": [\n");
    for(size_t i = 0; i < entries.size(); i++) {
        const auto &e = entries[i];
        fprintf(f, "    {\"name\": \"%s\", \"duration_ms\": %llu, \"depth\": %d}%s\n",
            e.name,
            (unsigned long long)e.durationMs,
            e.depth,
            i < entries.size() - 1 ? "," : "");
    }
    fprintf(f, "  ]\n}\n");

    fclose(f);
    fprintf(stdout, "Profile written to %s\n", path.c_str());
}

} // namespace SolveSpace
