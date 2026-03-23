#pragma once

#pragma once

#include <SDL.h>
#include <cstdint>
#include <cstring>
#include <string>

namespace sr_sound {

inline SDL_AudioDeviceID g_device = 0;
inline SDL_AudioSpec g_want{};
inline SDL_AudioSpec g_have{};
inline Uint8* g_buf = nullptr;
inline Uint32 g_buf_len = 0;
inline Uint32 g_pos = 0;
inline bool g_playing = false;
inline bool g_loop = false;
inline float g_volume = 1.0f; // 0.0..1.0

// simple callback: copies/loops 16-bit PCM; assumes g_want.format == AUDIO_S16LSB
inline void audio_callback(void* /*udata*/, Uint8* stream, int len) {
    std::memset(stream, 0, len);
    if (!g_playing || g_buf == nullptr || g_buf_len == 0) return;
    // guard
    Uint32 remaining = g_buf_len - g_pos;
    Uint32 tocopy = (Uint32)len;
    Uint8* outptr = stream;
    while (tocopy > 0) {
        if (remaining == 0) {
            if (g_loop) {
                g_pos = 0;
                remaining = g_buf_len;
            } else {
                g_playing = false;
                break;
            }
        }
        Uint32 chunk = (remaining < tocopy) ? remaining : tocopy;
        // apply simple volume for 16-bit signed samples
        if (g_volume >= 0.999f && g_volume <= 1.001f) {
            std::memcpy(outptr, g_buf + g_pos, chunk);
        } else {
            // volume scaling - assume 16-bit stereo/mono interleaved
            int16_t* src = reinterpret_cast<int16_t*>(g_buf + g_pos);
            int16_t* dst = reinterpret_cast<int16_t*>(outptr);
            Uint32 samples = chunk / sizeof(int16_t);
            float vol = g_volume;
            for (Uint32 i = 0; i < samples; ++i) {
                int v = static_cast<int>(src[i] * vol);
                if (v > 32767) v = 32767;
                if (v < -32768) v = -32768;
                dst[i] = static_cast<int16_t>(v);
            }
        }
        outptr += chunk;
        g_pos += chunk;
        tocopy -= chunk;
        remaining = g_buf_len - g_pos;
    }
}

// Initialize SDL audio and open a device. Output format: AUDIO_S16LSB.
inline bool init(int freq = 44100, int channels = 2, int samples = 4096) {
    if (SDL_InitSubSystem(SDL_INIT_AUDIO) < 0) return false;
    g_want.callback = audio_callback;
    g_want.userdata = nullptr;
    g_want.freq = freq;
    g_want.format = AUDIO_S8;
    g_want.channels = static_cast<Uint8>(channels);
    g_want.samples = static_cast<Uint16>(samples);
    g_device = SDL_OpenAudioDevice(nullptr, 0, &g_want, &g_have, SDL_AUDIO_ALLOW_FORMAT_CHANGE);
    if (g_device == 0) return false;
    // make sure callback will handle format we expect; if SDL changed format, convert on load
    return true;
}

// Load WAV file into a global buffer. Converts to device format if needed.
// Returns true on success.
inline bool loadWAV(const std::string& path) {
    if (g_device == 0) return false;
    SDL_AudioSpec spec = {};
    Uint8* wav_buf = nullptr;
    Uint32 wav_len = 0;
    if (SDL_LoadWAV(path.c_str(), &spec, &wav_buf, &wav_len) == nullptr) return false;
    // if format differs from desired, convert
    if (spec.format != g_have.format || spec.channels != g_have.channels || spec.freq != g_have.freq) {
        SDL_AudioCVT cvt;
        if (SDL_BuildAudioCVT(&cvt, spec.format, spec.channels, spec.freq,
                                   g_have.format, g_have.channels, g_have.freq) == 1) {
            cvt.len = static_cast<int>(wav_len);
            cvt.buf = static_cast<Uint8*>(SDL_malloc(cvt.len * cvt.len_mult));
            if (!cvt.buf) { SDL_FreeWAV(wav_buf); return false; }
            std::memcpy(cvt.buf, wav_buf, wav_len);
            if (SDL_ConvertAudio(&cvt) < 0) { SDL_free(cvt.buf); SDL_FreeWAV(wav_buf); return false; }
            // replace buffers
            SDL_FreeWAV(wav_buf);
            if (g_buf) { SDL_free(g_buf); g_buf = nullptr; g_buf_len = 0; }
            g_buf = cvt.buf;
            g_buf_len = static_cast<Uint32>(cvt.len_cvt);
        } else {
            // no conversion needed but BuildAudioCVT returned 0 (maybe identical) - just copy
            if (g_buf) { SDL_free(g_buf); g_buf = nullptr; g_buf_len = 0; }
            g_buf = static_cast<Uint8*>(SDL_malloc(wav_len));
            if (!g_buf) { SDL_FreeWAV(wav_buf); return false; }
            std::memcpy(g_buf, wav_buf, wav_len);
            g_buf_len = wav_len;
            SDL_FreeWAV(wav_buf);
        }
    } else {
        // same format: copy into managed buffer
        if (g_buf) { SDL_free(g_buf); g_buf = nullptr; g_buf_len = 0; }
        g_buf = static_cast<Uint8*>(SDL_malloc(wav_len));
        if (!g_buf) { SDL_FreeWAV(wav_buf); return false; }
        std::memcpy(g_buf, wav_buf, wav_len);
        g_buf_len = wav_len;
        SDL_FreeWAV(wav_buf);
    }
    g_pos = 0;
    return true;
}

// Play current buffer. If loopArg true, audio will loop.
inline void play(bool loopArg = false) {
    if (g_device == 0) return;
    SDL_LockAudioDevice(g_device);
    g_loop = loopArg;
    g_pos = 0;
    g_playing = true;
    SDL_UnlockAudioDevice(g_device);
    SDL_PauseAudioDevice(g_device, 0);
}

// Stop playback
inline void stop() {
    if (g_device == 0) return;
    SDL_LockAudioDevice(g_device);
    g_playing = false;
    g_pos = 0;
    SDL_UnlockAudioDevice(g_device);
    SDL_PauseAudioDevice(g_device, 1);
}

// Set volume 0.0..1.0
inline void setVolume(float v) {
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    if (g_device) SDL_LockAudioDevice(g_device);
    g_volume = v;
    if (g_device) SDL_UnlockAudioDevice(g_device);
}

// Close device and free buffers
inline void shutdown() {
    if (g_device) {
        SDL_CloseAudioDevice(g_device);
        g_device = 0;
    }
    if (g_buf) {
        SDL_free(g_buf);
        g_buf = nullptr;
        g_buf_len = 0;
    }
    SDL_QuitSubSystem(SDL_INIT_AUDIO);
}

} // namespace sr_sound