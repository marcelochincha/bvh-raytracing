#include <SDL2/SDL.h>
#include <SDL2/SDL_mixer.h>
#include <stdio.h>

int main(int argc, char *argv[]) {
    if (argc < 2) {
        printf("Uso: %s archivo.xm\n", argv[0]);
        return 1;
    }

    const char *filename = argv[1];

    // Inicializar SDL
    if (SDL_Init(SDL_INIT_AUDIO) < 0) {
        printf("Error SDL_Init: %s\n", SDL_GetError());
        return 1;
    }

    // Inicializar SDL_mixer con soporte para módulos
    int flags = MIX_INIT_MOD;  // o MIX_INIT_OGG | MIX_INIT_MP3 si quisieras más
    int initted = Mix_Init(flags);
    if ((initted & flags) != flags) {
        printf("Error Mix_Init: %s\n", Mix_GetError());
        // No hacemos return aún, SDL_mixer igual puede funcionar sin init()
    }

    if (Mix_OpenAudio(44100, MIX_DEFAULT_FORMAT, 2, 1024) == -1) {
        printf("Error Mix_OpenAudio: %s\n", Mix_GetError());
        return 1;
    }

    // Cargar el archivo XM
    Mix_Music *music = Mix_LoadMUS(filename);
    if (!music) {
        printf("Error Mix_LoadMUS: %s\n", Mix_GetError());
        return 1;
    }

    // Reproducir (loop = -1 significa repetir indefinidamente)
    if (Mix_PlayMusic(music, -1) == -1) {
        printf("Error Mix_PlayMusic: %s\n", Mix_GetError());
        Mix_FreeMusic(music);
        return 1;
    }

    printf("Reproduciendo %s... presiona ENTER para detener.\n", filename);
    getchar();

    // Limpiar
    Mix_HaltMusic();
    Mix_FreeMusic(music);
    Mix_CloseAudio();
    Mix_Quit();
    SDL_Quit();
    return 0;
}
