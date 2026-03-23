#define SDL_MAIN_HANDLED

#include <iostream>
#include <vector>
#include <array>

#include <SDL2/SDL_mixer.h>

#include <algorithm>
#include <math/sr_math.hpp>
#include <renderer/sr_renderer.hpp>
#include <renderer/sr_camera.hpp>
#include <renderer/sr_texture.hpp>
#include <renderer/sr_text.hpp>
#include <engine/sr_engine.hpp>

SDL_Window *window;
SDL_Renderer *renderer;
SDL_Texture *sdl_fb_texture;


#define W_WIDTH 440 * 2
#define W_HEIGHT 254 * 2
constexpr float TARGET_FPS = 90.0f;
constexpr float TARGET_DELTA_TIME_MS = 1000.0f / TARGET_FPS;


struct FDNReverb {
    static const int N = 4;
    std::vector<float> delayLines[N];
    int ptr[N] = {0};
    int sizes[N] = {1117, 1361, 1637, 1901};

    // Parámetros
    float decay = 0.1f;
    float damp = 0.4f;
    float lastOut[N] = {0};

    // Filtro High-Pass (Corta bajos de la batería)
    float hp_last_in = 0.0f;
    float hp_last_out = 0.0f;

    // Pre-Delay
    std::vector<float> preDelayBuffer;
    int preDelayPtr = 0;

    FDNReverb() {
        for(int i=0; i<N; i++) delayLines[i].resize(sizes[i], 0.0f);
        preDelayBuffer.resize(882, 0.0f); // 40ms aprox
    }

    float process(float input) {
        // SEGURIDAD INICIAL
        if (!std::isfinite(input)) input = 0.0f;

        // --- PASO 1: HIGH PASS (Limpieza de graves) ---
        float alpha = 0.85f; 
        float filteredInput = alpha * (hp_last_out + input - hp_last_in);
        hp_last_in = input;
        hp_last_out = filteredInput;

        // --- PASO 2: PRE-DELAY ---
        float delayedInput = preDelayBuffer[preDelayPtr];
        preDelayBuffer[preDelayPtr] = filteredInput;
        preDelayPtr = (preDelayPtr + 1) % preDelayBuffer.size();

        // --- PASO 3: LECTURA DE DELAYS + DAMPING ---
        float s[N];
        for(int i=0; i<N; i++) {
            float out = delayLines[i][ptr[i]];
            // Reset de seguridad si explota
            if (!std::isfinite(out)) { out = 0.0f; delayLines[i].assign(sizes[i], 0.0f); }
            
            lastOut[i] = out + damp * (lastOut[i] - out);
            s[i] = lastOut[i];
        }

        // --- PASO 4: MATRIZ DE HADAMARD (Mezcla) ---
        float f[N];
        f[0] = (s[0] + s[1] + s[2] + s[3]) * 0.5f * decay;
        f[1] = (s[0] - s[1] + s[2] - s[3]) * 0.5f * decay;
        f[2] = (s[0] + s[1] - s[2] - s[3]) * 0.5f * decay;
        f[3] = (s[0] - s[1] - s[2] + s[3]) * 0.5f * decay;

        for(int i=0; i<N; i++) {
            float val = delayedInput + f[i];
            delayLines[i][ptr[i]] = std::tanh(val); 
            ptr[i] = (ptr[i] + 1) % sizes[i];
        }

        // --- SALIDA ---
        float mix = (s[0] + s[1] + s[2] + s[3]) * 0.25f;
        return std::clamp(mix, -1.0f, 1.0f);
    }
};
FDNReverb globalReverb;
#define HALF_U16 32768.0f
void audioCallback(void *userdata, Uint8 *stream, int len)
{
    Uint16 *fBuffer = reinterpret_cast<Uint16 *>(stream);
    int totalSamples = len / sizeof(Uint16);

    // PARÁMETRO DE WETNESS (0.0f = Solo música, 1.0f = Solo Reverb)
    // Ajústalo entre 0.1f y 0.3f para ese efecto de "paz en las nubes"
    for (int i = 0; i < totalSamples; i++)
    {
        float drySample = ((float)fBuffer[i] - HALF_U16) / HALF_U16; // El sonido original (.xm)
        float wetSample = globalReverb.process(drySample);
        float mixedOutput = drySample + wetSample;
        fBuffer[i] = (Uint16)((mixedOutput + 1.0f) * HALF_U16); // Convertir back a Uint8
    }
}


const int audio_rate = 8192 * 2; // Baja fidelidad para ahorrar CPU
const Uint16 audio_type = AUDIO_U16; // 32-bit float audio

void init_sdl()
{
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO);
    window = SDL_CreateWindow(
        "sr_lec",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        W_WIDTH, W_HEIGHT, SDL_WINDOW_RESIZABLE);
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    SDL_RenderSetLogicalSize(renderer, W_WIDTH, W_HEIGHT);
    SDL_RenderSetIntegerScale(renderer, SDL_TRUE);
    sdl_fb_texture = SDL_CreateTexture(
        renderer,
        SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING,
        W_WIDTH, W_HEIGHT);

        
    // init audio
    // Abrir audio a 22kHz para ahorrar CPU en tu software renderer
    if (Mix_OpenAudio(audio_rate, audio_type, 1, 1024) < 0)
    {
        std::cerr << "SDL_mixer could not initialize! SDL_mixer Error: " << Mix_GetError() << std::endl;
        return;
    }

    printf("SDL_mixer initialized with Freq: %d\n", audio_rate);
    Mix_Init(MIX_INIT_OGG | MIX_INIT_MOD | MIX_INIT_MP3 | MIX_INIT_MID);
    Mix_Music *bgm = Mix_LoadMUS("res/music/w_chan.xm");
    if (!bgm)
    {
        std::cerr << "Failed to load background music! SDL_mixer Error: " << Mix_GetError() << std::endl;
        return;
    }

    // Registrar y reproducir con reverb post-mix
    // Mix_SetPostMix(MixingCallback, NULL);
    //if (Mix_PlayMusic(bgm, -1) == -1)
    //{
    //    std::cerr << "Failed to play music! SDL_mixer Error: " << Mix_GetError() << std::endl;
    //}

    // Change volume
    //Mix_VolumeMusic(MIX_MAX_VOLUME * 0.1f); // 20% volumen
    // set audio callback
    //Mix_SetPostMix(audioCallback, nullptr);
}

int main(int argc, char *argv[])
{

    init_sdl(); // SDL INIT
    bool running = true;
    bool debugMode = false;
    Engine *engine = engine_create(W_WIDTH, W_HEIGHT);

    engine_init(engine);

    const float DT = 1.0f / 60;
    float deltaTimeSeconds = DT;
    uint64_t lastTime, currentTime = SDL_GetTicks64();
    int frameCount = 0;
    // main loop
    while (running)
    {
        lastTime = currentTime;
        // FRAME START
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            engine_handle_events(engine, event, running);
        }

        //if debug mode wait for f2
        if (debugMode)
        {
            //wait for f2 to be pressed to proceed to the next frame
            while (true)
            {
                SDL_Event debugEvent;
                if (SDL_PollEvent(&debugEvent))
                {
                    engine_handle_events(engine, debugEvent, running);
                    if (debugEvent.type == SDL_KEYDOWN && debugEvent.key.keysym.scancode == SDL_SCANCODE_F2)
                    {
                        deltaTimeSeconds = DT; // Simular 60 FPS en modo debug para no depender del tiempo real
                        break;
                    }
                    if (debugEvent.type == SDL_QUIT)
                    {
                        running = false;
                        break;
                    }
                }
            }
        }
        if (frameCount < 5) // Solo los primeros 5 frames para no spamear la consola
        {
            printf("Frame %d: Frametime: %.4f seconds\n", frameCount, deltaTimeSeconds);
            frameCount++;
        }
        engine_update(engine, deltaTimeSeconds);
        engine_render(engine, sdl_fb_texture, deltaTimeSeconds);



        //  Render to screen
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, sdl_fb_texture, NULL, NULL);
        SDL_RenderPresent(renderer);

        // wait
        currentTime = SDL_GetTicks64();
        if (currentTime - lastTime < TARGET_DELTA_TIME_MS)
        {
            SDL_Delay(TARGET_DELTA_TIME_MS - (currentTime - lastTime));
            currentTime = SDL_GetTicks64();
        }
        deltaTimeSeconds = (currentTime - lastTime) / 1000.0f;

        // frame cap
    }

    // DIETARY CLEANUP
    SDL_DestroyTexture(sdl_fb_texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
