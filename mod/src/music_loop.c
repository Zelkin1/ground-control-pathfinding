// Music on repeat (`music_loop` in gcmod.ini). When a mission loads, the base game plays one of
// the tracks sound/soundtrack/track1.mpd to track8.mpd, chosen at random, as loading music
// (`SessionClient::StartLoadingMusic` 0x4a4c30), and asks its MP3 stream (`SE_StreamMp3`) to
// loop. Most mission scripts then start an ambient loop (sound/soundtrack/amb*.mpd: desert wind,
// jungle, winter) as the soundtrack of the mission (`CL_SoundTrack`). `StopLoadingMusic`
// (0x4a4d00), which runs at each update of the mission, fades the loading music out over 5 s.
//
// A hook on `StopLoadingMusic` keeps the loading music playing while the soundtrack plays no
// track or an ambient loop, so the music loops until the mission ends. It gives the music the
// volume of the ambient loop, so the music volume of the options and the lower volume while the
// message box of the mission shows a message act on it too. A music track that a script sets
// replaces the loading music as in the base game.
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "mod.h"
#include "game_object.h"

#define THISCALL __attribute__((thiscall))
#define STDCALL __attribute__((stdcall))

// `StopLoadingMusic`: a thiscall method of the session client without arguments. Its first
// instructions are push esi, mov esi, ecx, mov ecx, [esi] (5 bytes,
// decompilation/tools/gcdis.py dis).
#define STOP_LOADING_MUSIC_ADDRESS 0x4a4d00
#define STOP_LOADING_MUSIC_STOLEN_BYTES 5

// Fields of the session client (`SessionClient`), from `StopLoadingMusic` and
// `SessionClient::SessionUpdate` (0x4a6950), which passes the session to its update 0x4a39f0.
#define CLIENT_LOADING_MUSIC 0x0            // stream of the loading music, or NULL
#define CLIENT_IS_FADE_OUT_STARTED 0x4      // byte, not 0 once the loading music fades out
#define CLIENT_SESSION 0xac                 // the session of the mission (`CL_Session`)

// The soundtrack is a member of the session (`CL_Session::ParseSoundTrack` 0x4a30e0). Its play
// method 0x4b9710 stores the stream of the new track and its path, such as
// "sound/soundtrack/AmbDesertNight.mpd" (a game string: a pointer to its characters).
#define SESSION_SOUNDTRACK 0x24
#define SOUNDTRACK_CURRENT_TRACK 0x0        // stream of the current track, or NULL
#define SOUNDTRACK_TRACK_PATH 0x8

// Every ambient loop of the game has a file name that starts with these letters; the music tracks
// start with "track".
#define AMBIENT_FILE_PREFIX "amb"

// Fields of the stream (`SE_StreamMp3`). Its volume method 0x53c8b0 stores the volume as a
// DirectSound volume (hundredths of a decibel) and passes it to the buffer only while no fade-in
// or fade-out runs; the update thread of the stream passes it at the end of a fade.
#define STREAM_SOUND_BUFFER 0x10            // IDirectSoundBuffer of the stream
#define STREAM_IS_FADING 0x1c               // byte, not 0 while a fade runs
#define STREAM_VOLUME 0x34                  // 32-bit integer
#define SET_VOLUME_SLOT 0x3c                // byte offset of SetVolume in the buffer vtable

typedef void(THISCALL *StopLoadingMusicFunction)(void *session_client);
typedef long(STDCALL *SetVolumeFunction)(void *sound_buffer, long volume);

static StopLoadingMusicFunction stop_loading_music;   // trampoline to the original function
static const void *kept_stream;       // loading music whose keeping is logged
static const void *replaced_stream;   // loading music whose replacement is logged


// `is_ambient_track` tells whether the soundtrack file at `track_path` is an ambient loop.
static bool is_ambient_track(const char *track_path) {
    const char *slash = strrchr(track_path, '/');
    const char *file_name = slash ? slash + 1 : track_path;
    return _strnicmp(file_name, AMBIENT_FILE_PREFIX, strlen(AMBIENT_FILE_PREFIX)) == 0;
}


// `copy_stream_volume` gives `target_stream` the volume of `source_stream`, as the volume method
// of the game would.
static void copy_stream_volume(void *target_stream, const void *source_stream) {
    int volume = read_int(source_stream, STREAM_VOLUME);
    if (read_int(target_stream, STREAM_VOLUME) == volume)
        return;
    write_int(target_stream, STREAM_VOLUME, volume);
    if (read_byte(target_stream, STREAM_IS_FADING))
        return;
    void *sound_buffer = read_pointer(target_stream, STREAM_SOUND_BUFFER);
    const void *vtable = read_pointer(sound_buffer, 0);
    SetVolumeFunction set_volume = read_pointer(vtable, SET_VOLUME_SLOT);
    set_volume(sound_buffer, volume);
}


// `hook_stop_loading_music` keeps the loading music of `session_client` playing beside an ambient
// loop of the soundtrack, or without a soundtrack track, and runs the original function otherwise.
static void THISCALL hook_stop_loading_music(void *session_client) {
    void *stream = read_pointer(session_client, CLIENT_LOADING_MUSIC);
    bool is_fade_out_started = read_byte(session_client, CLIENT_IS_FADE_OUT_STARTED) != 0;
    if (!stream || is_fade_out_started) {
        stop_loading_music(session_client);
        return;
    }

    // A music track that a mission script set replaces the loading music, as in the base game.
    const void *session = read_pointer(session_client, CLIENT_SESSION);
    const void *soundtrack = &((const uint8_t *)session)[SESSION_SOUNDTRACK];
    const void *track = read_pointer(soundtrack, SOUNDTRACK_CURRENT_TRACK);
    const char *track_path = read_pointer(soundtrack, SOUNDTRACK_TRACK_PATH);
    if (track && !is_ambient_track(track_path)) {
        if (stream != replaced_stream)
            write_log("music loop: the soundtrack %s replaces the loading music", track_path);
        replaced_stream = stream;
        stop_loading_music(session_client);
        return;
    }

    // Without a soundtrack track, the music keeps the volume that it started with.
    if (stream != kept_stream)
        write_log("music loop: loading music kept");
    kept_stream = stream;
    if (track)
        copy_stream_volume(stream, track);
}


// `install_music_loop_hook` hooks `StopLoadingMusic` with `install_hook` when `music_loop` is on;
// otherwise the game function stays as it is.
void install_music_loop_hook(InstallHookFunction install_hook, const ModConfig *config) {
    if (!config->is_music_loop_on)
        return;
    kept_stream = NULL;
    replaced_stream = NULL;
    stop_loading_music = install_hook(STOP_LOADING_MUSIC_ADDRESS, STOP_LOADING_MUSIC_STOLEN_BYTES,
                                      (const void *)hook_stop_loading_music);
    if (!stop_loading_music)
        write_log("hook of the loading music failed: the music fades out as in the base game");
}
