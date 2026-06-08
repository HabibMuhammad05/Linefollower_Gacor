#ifndef MAPPING_H
#define MAPPING_H

/*
  MAPPING JALUR ROBOT
  ====================================================================
  File ini KHUSUS buat atur keputusan robot di tiap persimpangan yang
  dilewati (termasuk aksi servo/gripper), biar gampang diedit tanpa
  bongkar file utama.

  FORMAT BARU - tiap baris = STRUCT { aksi, durasiMs }:
    - durasiMs = 0   -> pakai nilai DEFAULT dari konstanta di bawah
                        (TURN_KIRI_MS, STOP_SESAAT_MS, dst) sesuai jenis aksinya
    - durasiMs = angka (misal 800) -> OVERRIDE, khusus buat baris itu aja,
                        ga ngaruh ke baris/simpang lain yang aksinya sama
    - Servo (ACT_SERVO_*), ACT_LURUS, ACT_BERHENTI GA PAKAI durasiMs -
      isi 0 aja (bakal diabaikan)

  CARA PAKAI:
  - Isi array jalurMap[] di bawah, urut dari persimpangan PERTAMA yang
    bakal ketemu sampai terakhir. Index 0 = simpang ke-1, dst.
  - Kalau robot ketemu simpang MELEBIHI panjang array ini (mapping-nya
    udah abis tapi robot masih jalan), default-nya ACT_LURUS -> robot
    tetap jalan lurus ngikutin PD seperti biasa, PERSIS behavior
    sebelum fitur mapping ini ada.

  AKSI YANG TERSEDIA:
    ACT_LURUS        -> jalan lurus normal, ga ada maneuver spesial
    ACT_KIRI         -> pivot belok kiri di tempat
    ACT_KANAN        -> pivot belok kanan di tempat
    ACT_PUTAR        -> putar balik 180 derajat
    ACT_BERHENTI     -> berhenti total, dianggap SAMPAI TUJUAN -> masuk
                        state FINISH lalu auto balik IDLE (beda dari
                        tombol STOP manual yang nempel merah nunggu user)
    ACT_STOP_SESAAT  -> berhenti SEBENTAR doang, abis itu LANJUT jalan
                        lagi ngikutin PD (bukan finish, bukan idle)
    ACT_MUNDUR       -> jalan mundur sedikit
    ACT_SERVO_NAIK   -> gripper naik
    ACT_SERVO_TURUN  -> gripper turun
    ACT_SERVO_BUKA   -> jepit dibuka
    ACT_SERVO_TUTUP  -> jepit ditutup

  Contoh di bawah cuma ilustrasi urutan - GANTI sesuai jalur arena asli.
  ====================================================================
*/

enum TurnAction {
  ACT_LURUS, ACT_KIRI, ACT_KANAN, ACT_PUTAR, ACT_BERHENTI,
  ACT_STOP_SESAAT, ACT_MUNDUR,
  ACT_SERVO_NAIK, ACT_SERVO_TURUN, ACT_SERVO_BUKA, ACT_SERVO_TUTUP
};

struct MapEntry {
  TurnAction action;
  unsigned long durasiMs;  // 0 = pakai default (lihat konstanta di bawah)
};

// ------------------------ EDIT DI SINI ------------------------
const MapEntry jalurMap[] = {
  { ACT_KIRI,        0    },  // simpang ke-1: belok kiri, durasi default (350ms)
  { ACT_SERVO_TURUN, 0    },  // simpang ke-2: gripper turun (contoh testing servo)
  { ACT_STOP_SESAAT, 800  },  // simpang ke-3: berhenti 800ms (override dari default 500ms), lalu lanjut jalan
  { ACT_BERHENTI,    0    },  // simpang ke-4: berhenti total, dianggap sampai tujuan
};
// ----------------------------------------------------------------

const uint8_t JALUR_MAP_COUNT = sizeof(jalurMap) / sizeof(jalurMap[0]);

// ---- Default durasi tiap tipe aksi (dipakai kalau durasiMs di jalurMap = 0) ----
// WAJIB dikalibrasi manual di lapangan - arena beda, waktu pivot/mundur juga beda.
const unsigned long TURN_KIRI_MS   = 350; // durasi pivot kiri  default (~90 derajat)
const unsigned long TURN_KANAN_MS  = 350; // durasi pivot kanan default (~90 derajat)
const unsigned long TURN_PUTAR_MS  = 600; // durasi u-turn      default (~180 derajat)
const unsigned long STOP_SESAAT_MS = 500; // durasi berhenti sebentar default
const unsigned long MUNDUR_MS      = 300; // durasi jalan mundur default

// ---- Kecepatan motor buat manuver ----
const int TURN_SPEED   = 200; // kecepatan pivot belok/putar (0-255)
const int MUNDUR_SPEED = 120; // kecepatan jalan mundur (0-255)

// durasi animasi LED "nyempit ke tengah" sebelum masuk state FINISH pas ACT_BERHENTI
const unsigned long BERHENTI_PREVIEW_MS = 500;

// ====================================================================
// CATATAN RENCANA (BELUM DIIMPLEMENTASI - cuma buat persiapan):
// 1. Beberapa PROFIL mapping (misal jalurMap_A, jalurMap_B, jalurMap_C),
//    masing-masing array keputusan sendiri buat rute berbeda, dipilih
//    sebelum mulai jalan (misal lewat tombol tambahan/dip-switch).
// 2. Servo (misal ACT_SERVO_TURUN) dipicu LANGSUNG bareng aksi gerak di
//    simpang yang sama (bukan gantian satu-satu kayak sekarang) - versi
//    sekarang sengaja 1 aksi per simpang dulu biar simpel & gampang dites.
// ====================================================================

#endif
