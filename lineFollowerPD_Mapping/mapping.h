#ifndef MAPPING_H
#define MAPPING_H

/*
  MAPPING JALUR ROBOT
  ====================================================================
  File ini KHUSUS buat atur keputusan robot di tiap persimpangan yang
  dilewati, biar gampang diedit tanpa bongkar file utama.

  CARA PAKAI:
  - Isi array jalurMap[] di bawah, urut dari persimpangan PERTAMA yang
    bakal ketemu sampai terakhir.
  - Index 0 = simpang ke-1, index 1 = simpang ke-2, dst.
  - Kalau robot ketemu simpang MELEBIHI panjang array ini (mapping-nya
    udah abis tapi robot masih jalan), default-nya ACT_LURUS -> robot
    tetap jalan lurus ngikutin PD seperti biasa, PERSIS behavior
    sebelum fitur mapping ini ada (ga ada yang berubah kalau ga di-map).

  AKSI YANG TERSEDIA:
    ACT_LURUS    -> lanjut jalan lurus normal, ga ada maneuver spesial
    ACT_KIRI     -> pivot belok kiri di tempat
    ACT_KANAN    -> pivot belok kanan di tempat
    ACT_PUTAR    -> putar balik 180 derajat
    ACT_BERHENTI -> berhenti total (dianggap sudah sampai tujuan)

  Contoh di bawah: simpang ke-1 belok kiri, simpang ke-2 berhenti.
  Tinggal tambah/ubah baris sesuai jalur arena yang sebenarnya.
  ====================================================================
*/

enum TurnAction { ACT_LURUS, ACT_KIRI, ACT_KANAN, ACT_PUTAR, ACT_BERHENTI };

// ------------------------ EDIT DI SINI ------------------------
const TurnAction jalurMap[] = {
  ACT_KIRI,      // simpang ke-1 -> belok kiri
  ACT_KIRI,      // simpang ke-1 -> belok kiri
  ACT_KIRI,      // simpang ke-1 -> belok kiri
  ACT_PUTAR,
  ACT_KIRI,      // simpang ke-1 -> belok kiri
  ACT_KANAN,
  ACT_BERHENTI,  // simpang ke-2 -> berhenti (contoh: nyampe tujuan)
};
// ----------------------------------------------------------------

const uint8_t JALUR_MAP_COUNT = sizeof(jalurMap) / sizeof(jalurMap[0]);

// ---- Tuning maneuver (kecepatan & durasi pivot belok/putar) ----
// WAJIB dikalibrasi manual di lapangan: arena beda, waktu pivot buat
// belok ~90 derajat juga beda tergantung berat robot & lebar roda.
const int           TURN_SPEED    = 200; // kecepatan pivot (0-255)
const unsigned long TURN_KIRI_MS  = 350; // durasi pivot kiri  (~90 derajat)
const unsigned long TURN_KANAN_MS = 350; // durasi pivot kanan (~90 derajat)
const unsigned long TURN_PUTAR_MS = 600; // durasi u-turn      (~180 derajat)

// durasi animasi LED "nyempit ke tengah" sesaat sebelum masuk state FINISH
// pas ketemu ACT_BERHENTI (robot udah berhenti total, cuma nunggu animasi)
const unsigned long BERHENTI_PREVIEW_MS = 1000;

// ====================================================================
// CATATAN RENCANA (BELUM DIIMPLEMENTASI - cuma buat persiapan):
// Ke depan mau ada beberapa PROFIL mapping (misal jalurMap_A, jalurMap_B,
// jalurMap_C), masing-masing array keputusan sendiri buat rute yang
// beda, dan robot bisa milih profil mana yang dipakai (misal lewat
// tombol tambahan atau dip-switch) sebelum mulai jalan. Versi sekarang
// masih 1 array tunggal (jalurMap[]) dulu, sengaja disederhanain biar
// gampang dites & diedit.
// ====================================================================

#endif
