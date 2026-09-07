#include "checked_json.h"
#include <cassert>
#include <iostream>

int main() {
    String output;
    CheckedJson writer(output);
    writer.field("text", "a\"\\\n\t\x01\xc3\xa9", true);
    writer.field("number", "42", false);
    writer.field("enabled", "true", false);
    assert(writer.finish());
    assert(output == "{\"text\":\"a\\\"\\\\\\u000a\\u0009\\u0001\xc3\xa9\",\"number\":42,\"enabled\":true}");

    String::allocationLimit = 100;
    CheckedJson noMemory(output);
    noMemory.field("text", "value", true);
    assert(!noMemory.finish());
    assert(output.empty());

    String::allocationLimit = 2048;
    CheckedJson exhausted(output);
    exhausted.field("long", String(3000, 'x'), true);
    exhausted.field("next", "true", false);
    assert(!exhausted.finish());
    assert(output.empty());

    String::allocationLimit = std::string::npos;
    CheckedJson recovered(output);
    recovered.field("ok", "true", false);
    assert(recovered.finish());
    assert(output == "{\"ok\":true}");
    std::cout << "Checked JSON: escaping, reserve/append failure, recovery OK\n";
}
