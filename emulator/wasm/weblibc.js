// shims for weblibc functions that can't reasonably be implemented in pure C

// These are the functions that we would expect to call from JavaScript.
//int unlink(const char *pathname);
//char *getcwd(char *buf, size_t size);
//int open(const char *pathname, int flags, ...); //with mode_t as the first parameter after the ... potentially.
//int creat(const char *pathname, mode_t mode);
//off_t lseek(int fd, off_t offset, int whence);
//ssize_t read(int fd, void *buf, size_t count);
//ssize_t write(int fd, const void *buf, size_t count);
//int close(int fd);
//char * getenv(const char *name)
//void exit(int reason);
//void * sbrk(int size);

// gettimeofday
// struct tm *localtime(const time_t *timep);
// time_t time(time_t* tloc);

// int clock_getres(clockid_t clockid, struct timespec *res);
// int clock_gettime(clockid_t clockid, struct timespec *tp);
// int clock_settime(clockid_t clockid, const struct timespec *tp);


//////////////////////////////////////////////////////////////////////////////
// dlsym.h

//These function will be nerfed in weblibc... Or maybe?  Eh, remains to be seen.
//void *dlsym(void *handle, const char *symbol);
//void *dlopen(const char *filename, int flags);
//int dlclose(void *handle);

// WebAssembly.Memory page size
const PAGE_SIZE = 64 * 1024;

const utfDecoder = new TextDecoder("utf-8");

let memory;
let HEAPU8;
let HEAPU32;
let HEAP32;
let HEAP64;

let heap_base;
let heap_top;

let argc = 0;
let argv = [];
let argvPtrs = [];

let env = {};
let envPtrMap = {};

function exit() {
    console.log("EXIT");
    console.trace();
}

function brk(addr) {
    if (addr > heap_top) {
        if (sbrk(addr - heap_top) != -1) {
            return 0;
        } else {
            return -1;
        }
    }

    return 0;
}

function sbrk(increment) {
    let result = heap_top;
    let newHeapTop = heap_top + increment;

    // Calculate total number of pages needed to store the heap
    // | 0 is short for floor()
    let curPageCount = (memory.buffer.byteLength / PAGE_SIZE) | 0;
    let pagesNeeded = ((newHeapTop + PAGE_SIZE - 1) / PAGE_SIZE) | 0;
    if (pagesNeeded > curPageCount) {
        try {
            memory.grow(pagesNeeded - curPageCount);
            HEAPU8 = new Uint8Array(memory);
            HEAP32 = new Int32Array(memory);
            HEAP64 = new BigInt64Array(memory);
        } catch (e) {
            // couldn't grow memory
            return -1;
        }
    }

    heap_top = newHeapTop;
    return result;
}

function fmod(x, y) {
    return x % y;
}

function unlink(pathname) {
    console.log("unlink", pathname);
    return 0;
}

function getcwd(buf, size) {
    if (size > 1) {
        HEAPU8[buf] = 47;
        HEAPU8[buf + 1] = 0;
    }

    return buf;
}

function getenv(name) {
    const result = envPtrMap[name];
    if (result) {
        return result;
    } else {
        return 0;
    }
}

function gettimeofday(tv, tz) {
    // int gettimeofday(struct timeval *tv, struct timezone *tz)
    return 0;
}

function settimeofday(tv, tz) {
    // int settimeofday(const struct timeval *tv, const struct timezone *tz)
    return 0;
}

function localtime(timep) {
    // struct tm *localtime(const time_t *timep)
    return 0;
}

function time(tloc) {
    // time_t time(time_t *tloc)
    return Math.floor(new Date().getTime() / 1000);
}

function clock_getres(clockid, res) {
    return 0;
}

function clock_gettime(clockid, tp) {
    let sec = 0;
    let nsec = 0;

    if (clockid == 1 && typeof performance != "undefined") {
        // performance clock is actually guaranteed to be monotonic, so use that
        let perfTime = performance.timeOrigin + performance.now();
        nsec = (perfTime % 1000) * 1000000;
        sec = (perfTime / 1000) | 0;
    } else if (typeof Temporal != "undefined" && typeof Temporal.Now != "undefined") {
        // Temporal isn't widely supported but has better resolution in theory
        nsec = Temporal.Now.instant().epochNanoseconds;
        sec = Math.floor(nsec / 1e6);
        nsec %= 1e6;
    } else {
        // good old Date everyone supports date
        let dateTime = Date.now();
        nsec = (dateTime % 1000) * 1000000;
        sec = (dateTime / 1000) | 0;
    }

    // Ok, I _think_ this is how this works?
    // HEAP32 is 4 bytes wide, and a struct timespec is an int64+int32, so...
    let tpVoidOffset = tp  * 12;
    let tpSecOffset = tpVoidOffset >> 3;
    let tpNsecOffset = (tpVoidOffset + 8) >> 2;

    console.log(typeof sec, sec);
    HEAP64[tpSecOffset] = BigInt(sec);
    HEAP32[tpNsecOffset] = nsec;
    
    return 0;
}

function clock_settime(clockid, tp) {
    return 0;
}

function nanosleep(req, rem) {
    return 0;
}

function _wlc_stdoutWrite(file, buf, n) {
    console.log(utfDecoder.decode(HEAPU8.subarray(buf, buf + n)));
    return n;
}

function _wlc_stderrWrite(file, buf, n) {
    console.error(utfDecoder.decode(HEAPU8.subarray(buf, buf + n)));
    return n;
}

function _wlc_stdinRead(buf, n) {
    console.log("stdinRead()");
    return 0;
}

function printf() {
    console.log("printf()");
}

const debugImports = true;
function _wrapLogFn(name, prefix, fn) {
    if (debugImports) {
        return function(...args) {
            console.log(">>>" + prefix + name + "(" + args.join(", ") + ")");
            let result = fn(...args);
            console.log("<<<" + prefix + name);
            return result;
        };
    } else {
        return fn;
    }
}

// Exports all the functions, using a provided one to override the default if given
function addImport(name, fn, imports, settings) {
    if (settings && typeof settings.overrides != "undefined" && name in settings.overrides) {
        imports.env[name] = _wrapLogFn(name, "!", settings.overrides[name]);
    } else {
        imports.env[name] = _wrapLogFn(name, " ", fn);
    }
}

function setupArgv(argvArr) {
    let len = 0;
    for (let arg of argvArr) {
        // account for the text, NUL, and a pointer to it
        len += (""+arg).length + 1 + 4;
    }
    
    // make sure len is a multiple of 4 bytes
    len = ((len + 3) / 4 | 0) * 4;
    let start = sbrk(len);

    // skip past the pointers for the raw data
    let cur = start + 4 * argvArr.length;
    // write the string values
    for (let arg of argvArr) {
        argvPtrs.push(cur);
        let strArg = (""+arg);
        for (let char of strArg) {
            HEAPU8[cur++] = char.charCodeAt(0) & 0xFF;
        }
        HEAPU8[cur++] = 0;
    }

    // now write the pointers to the start
    argv = start;
    for (let ptr of argvPtrs) {
        HEAPU32[start >> 2] = ptr;
        start += 4;
    }

    argc = argvArr.length;
}

function setupEnv(envObj) {
    let len = 0;
    for (let key in envObj) {
        let val = envObj[key];

        // account for the text, NUL, and a pointer to it
        len += (""+val).length + 1;
    }

    // make sure len is a multiple of 4 bytes
    len = ((len + 3) / 4 | 0) * 4;
    let start = sbrk(len);
    
    // skip past the pointers for the raw data
    let cur = start + 4 * envObj.length;
    // write the string values
    for (let key in envObj) {
        let val = (""+envObj[key]);

        envPtrMap[key] = cur;

        for (let char of val) {
            HEAPU8[cur++] = char.charCodeAt(0) & 0xFF;
        }
        HEAPU8[cur++] = 0;
    }
}

export function getArgv() {
    if (argvPtrs.length > 0) {
        return argvPtrs[0];
    }

    return 0;
}

export function postInstantiate(instance) {
    heap_base = instance.exports.__heap_base;
    heap_top = heap_base;
    console.log("set heap_top to", heap_top);

    setupArgv(argv);
    setupEnv(env);

    instance.exports.__std_files_init();

    return { argc, argv };
}

export default function configure(imports, settings) {
    memory = imports.env.memory;
    HEAPU8 = new Uint8Array(memory.buffer);
    HEAPU32 = new Uint32Array(memory.buffer);
    HEAP32 = new Int32Array(memory.buffer);
    HEAP64 = new BigInt64Array(memory.buffer);

    if (settings && typeof settings.argv != "undefined") {
        argv = settings.argv;
    }

    if (settings && typeof settings.env != "undefined") {
        env = settings.env;
    }

    addImport("exit", exit, imports, settings);

    addImport("sbrk", sbrk, imports, settings);
    addImport("brk", brk, imports, settings);

    addImport("cos", Math.cos, imports, settings);
    addImport("cosf", Math.cos, imports, settings);
    addImport("sin", Math.sin, imports, settings);
    addImport("sinf", Math.sin, imports, settings);
    addImport("tan", Math.tan, imports, settings);
    addImport("tanf", Math.tan, imports, settings);
    addImport("atan2", Math.atan2, imports, settings);
    addImport("atan2f", Math.atan2, imports, settings);
    addImport("ceil", Math.ceil, imports, settings);
    addImport("ceilf", Math.ceil, imports, settings);
    addImport("floor", Math.floor, imports, settings);
    addImport("floorf", Math.floor, imports, settings);
    addImport("round", Math.round, imports, settings);
    addImport("roundf", Math.round, imports, settings);
    addImport("fabs", Math.abs, imports, settings);
    addImport("fabsf", Math.abs, imports, settings);
    addImport("sqrt", Math.sqrt, imports, settings);
    addImport("sqrtf", Math.sqrt, imports, settings);
    addImport("exp", Math.exp, imports, settings);
    addImport("expf", Math.exp, imports, settings);
    addImport("log", Math.log, imports, settings);
    addImport("logf", Math.log, imports, settings);
    addImport("pow", Math.pow, imports, settings);
    addImport("powf", Math.pow, imports, settings);
    addImport("fmod", fmod, imports, settings);
    addImport("fmodf", fmod, imports, settings);

    addImport("unlink", unlink, imports, settings);
    addImport("getcwd", getcwd, imports, settings);
    addImport("getenv", getenv, imports, settings);

    addImport("gettimeofday", gettimeofday, imports, settings);
    addImport("settimeofday", settimeofday, imports, settings);

    addImport("localtime", localtime, imports, settings);
    addImport("time", time, imports, settings);
    addImport("clock_getres", clock_getres, imports, settings);
    addImport("clock_gettime", clock_gettime, imports, settings);
    addImport("clock_settime", clock_settime, imports, settings);
    addImport("nanosleep", nanosleep, imports, settings);

    addImport("_wlc_stdoutWrite", _wlc_stdoutWrite, imports, settings);
    addImport("_wlc_stderrWrite", _wlc_stderrWrite, imports, settings);
    addImport("_wlc_stdinRead", _wlc_stdinRead, imports, settings);

    //addImport("printf", printf, imports, settings);
}
