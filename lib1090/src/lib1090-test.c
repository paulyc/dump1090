#include <lib1090.h>
#include <stdio.h>

const uint8_t *testMsg = "\x8f\xa6\xc7\x81\x58\x4f\x23\x35\x45\x84\x0e\x4e\xf5\x92";

int main(int argc, char **argv) {

    int res = lib1090Init(0.0f, 0.0f, 0.0f);
    if (res != 0) {
        fprintf(stderr, "Got error %d from lib1090Init\n", res);
        return res;
    }
    usleep(10000000);
    res = lib1090RunThread(NULL);
    if (res != 0) {
        fprintf(stderr, "Got error %d from lib1090RunThread\n", res);
        return res;
    }
    struct modesMessage mm;
    res = lib1090HandleFrame(&mm, (uint8_t *)testMsg, 0);
    if (res != 0) {
        fprintf(stderr, "Got error %d from lib1090HandleFrame\n", res);
        return res;
    }
    usleep(10000000);
    res = lib1090Uninit(NULL);
    if (res != 0) {
        fprintf(stderr, "Got error %d from lib1090Uninit\n", res);
        return res;
    }
    return 0;
}
