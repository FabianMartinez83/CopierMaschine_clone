
// CopierMaschine clone created by Fabian Martinez
// https://github.com/FabianMartinez    
// This code is licensed under the MIT License.

/*
============
MIT License
============

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.







*/







#include <cmath>
#include <cstring>
#include <distingnt/api.h>

// --- Constants for buffer and scale definitions ---
#define ASR_BUF_SIZE 64 // Max buffer length for the analog shift register (ASR)
#define NUM_STAGES 4    // Number of output stages (A, B, C, D)
#define NUM_STANDARD_SCALES 16
#define NUM_EXOTIC_SCALES 117
#define NUM_SCALES (NUM_STANDARD_SCALES + NUM_EXOTIC_SCALES )
#define NUM_BYTEBEAT_EQNS 16
#define SCALE_MAX_LEN 20 // Maximum number of notes in a scale

// --- Integer Sequence definitions ---
#define NUM_INTSEQ 10
#define INTSEQ_MAX_LEN 128

// Integer sequences from Quantermain (O_C firmware)
static const int intseq_pi[INTSEQ_MAX_LEN] = {
    3,1,4,1,5,9,2,6,5,3,5,8,9,7,9,3,2,3,8,4,6,2,6,4,3,3,8,3,2,7,9,5,
    0,2,8,8,4,1,9,7,1,6,9,3,9,9,3,7,5,1,0,5,8,2,0,9,7,4,9,4,4,5,9,2,
    3,0,7,8,1,6,4,0,6,2,8,6,2,0,8,9,9,8,6,2,8,0,3,4,8,2,5,3,4,2,1,1,
    7,0,6,7,9,8,2,1,4,8,0,8,6,5,1,3,2,8,2,3,0,6,6,4,7,0,9,3,8,4,4,6
};
static const int intseq_vanEck[INTSEQ_MAX_LEN] = {
    0,0,1,0,2,0,2,2,1,6,5,5,7,6,7,9,8,8,10,9,11,10,12,11,13,12,14,13,15,14,16,15,
    17,16,18,17,19,18,20,19,21,20,22,21,23,22,24,23,25,24,26,25,27,26,28,27,29,28,30,29,31,30,32,31,
    33,32,34,33,35,34,36,35,37,36,38,37,39,38,40,39,41,40,42,41,43,42,44,43,45,44,46,45,47,46,48,47,
    49,48,50,49,51,50,52,51,53,52,54,53,55,54,56,55,57,56,58,57,59,58,60,59,61,60,62,61,63,62,64,63
};
static const int intseq_ssdn[INTSEQ_MAX_LEN] = {
    0,1,4,9,1,2,5,10,2,5,10,2,5,10,2,5,10,2,5,10,2,5,10,2,5,10,2,5,10,2,5,10,
    2,5,10,2,5,10,2,5,10,2,5,10,2,5,10,2,5,10,2,5,10,2,5,10,2,5,10,2,5,10,2,5,
    10,2,5,10,2,5,10,2,5,10,2,5,10,2,5,10,2,5,10,2,5,10,2,5,10,2,5,10,2,5,10,2,
    5,10,2,5,10,2,5,10,2,5,10,2,5,10,2,5,10,2,5,10,2,5,10,2,5,10,2,5,10,2,5,10
};
static const int intseq_dress[INTSEQ_MAX_LEN] = {
    0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,
    32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,
    64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,
    96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127
};
static const int intseq_pninf[INTSEQ_MAX_LEN] = {
    0,1,-1,2,0,1,-2,3,1,0,-1,2,-3,4,2,1,0,-1,3,-4,5,3,2,1,0,-1,4,-5,6,4,3,2,
    1,0,5,-6,7,5,4,3,2,1,6,-7,8,6,5,4,3,2,7,-8,9,7,6,5,4,3,8,-9,10,8,7,6,
    5,4,9,-10,11,9,8,7,6,5,10,-11,12,10,9,8,7,6,11,-12,13,11,10,9,8,7,12,-13,14,12,11,10,
    9,8,13,-14,15,13,12,11,10,9,14,-15,16,14,13,12,11,10,15,-16,17,15,14,13,12,11,16,-17,18,16,15,14
};
static const int intseq_dsum[INTSEQ_MAX_LEN] = {
    0,1,2,3,4,5,6,7,8,9,1,2,3,4,5,6,7,8,9,1,2,3,4,5,6,7,8,9,1,2,3,4,
    5,6,7,8,9,1,2,3,4,5,6,7,8,9,1,2,3,4,5,6,7,8,9,1,2,3,4,5,6,7,8,9,
    1,2,3,4,5,6,7,8,9,1,2,3,4,5,6,7,8,9,1,2,3,4,5,6,7,8,9,1,2,3,4,5,
    6,7,8,9,1,2,3,4,5,6,7,8,9,1,2,3,4,5,6,7,8,9,1,2,3,4,5,6,7,8,9,1
};
static const int intseq_dsum4[INTSEQ_MAX_LEN] = {
    0,1,2,3,1,2,3,1,2,3,1,2,3,1,2,3,1,2,3,1,2,3,1,2,3,1,2,3,1,2,3,1,
    2,3,1,2,3,1,2,3,1,2,3,1,2,3,1,2,3,1,2,3,1,2,3,1,2,3,1,2,3,1,2,3,
    1,2,3,1,2,3,1,2,3,1,2,3,1,2,3,1,2,3,1,2,3,1,2,3,1,2,3,1,2,3,1,2,
    3,1,2,3,1,2,3,1,2,3,1,2,3,1,2,3,1,2,3,1,2,3,1,2,3,1,2,3,1,2,3,1
};
static const int intseq_dsum5[INTSEQ_MAX_LEN] = {
    0,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,
    4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,
    4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,
    4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3
};
static const int intseq_cdn2[INTSEQ_MAX_LEN] = {
    0,-2,-4,-6,-8,-10,-12,-14,-16,-18,-20,-22,-24,-26,-28,-30,-32,-34,-36,-38,-40,-42,-44,-46,-48,-50,-52,-54,-56,-58,-60,-62,
    -64,-66,-68,-70,-72,-74,-76,-78,-80,-82,-84,-86,-88,-90,-92,-94,-96,-98,-100,-102,-104,-106,-108,-110,-112,-114,-116,-118,-120,-122,-124,-126,
    -128,-130,-132,-134,-136,-138,-140,-142,-144,-146,-148,-150,-152,-154,-156,-158,-160,-162,-164,-166,-168,-170,-172,-174,-176,-178,-180,-182,-184,-186,-188,-190,
    -192,-194,-196,-198,-200,-202,-204,-206,-208,-210,-212,-214,-216,-218,-220,-222,-224,-226,-228,-230,-232,-234,-236,-238,-240,-242,-244,-246,-248,-250,-252,-254
};
static const int intseq_frcti[INTSEQ_MAX_LEN] = {
    0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,4,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,5,
    0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,4,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,6,
    0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,4,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,7,
    0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,4,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,8
};

static const int* intseq_tables[NUM_INTSEQ] = {
    intseq_pi, intseq_vanEck, intseq_ssdn, intseq_dress, intseq_pninf,
    intseq_dsum, intseq_dsum4, intseq_dsum5, intseq_cdn2, intseq_frcti
};

static const char* intseq_names[NUM_INTSEQ] = {
    "pi", "vnEck", "ssdn", "Dress", "PNinf", "Dsum", "Dsum4", "Dsum5", "CDn2", "Frcti"
};

static const char* intseq_cv1_dest_names[] = {
    "mult/att", "seq", "strt", "len", "strd", "mod"
};
#define NUM_INTSEQ_CV1_DEST 6

// --- All scale names (standard + exotic) ---
static const char* all_scale_names[NUM_SCALES] = {
    // Standard scales
    "Major", "Minor", "Harmonic Minor", "Melodic Minor", "Mixolydian", "Dorian", "Lydian", "Phrygian",
    "Aeolian", "Locrian", "Maj Pent", "Min Pent", "Whole Tone", "Octatonic HW", "Octatonic WH", "Ionian",

    // Exotic scales (names must match the order and count of your exotic scales)
    "Blues Major", "Blues Minor", "Folk", "Japanese", "Gamelan", "Gypsy", "Arabian", "Flamenco",
    "Whole Tone (Exotic)", "Pythagorean", "1/4-EB", "1/4-E", "1/4-EA", "Bhairav", "Gunakri", "Marwa",
    "Shree", "Purvi", "Bilawal", "Yaman", "Kafi", "Bhimpalasree", "Darbari", "Rageshree",
    "Khamaj", "Mimal", "Parameshwari", "Rangeshwari", "Gangeshwari", "Kameshwari", "Pa_Kafi", "Natbhairav",
    "M_Kauns", "Bairagi", "B_Todi", "Chandradeep", "Kaushik_Todi", "Jogeshwari",
    "Tartini-Vallotti", "13/22-tET", "13/19-tET", "Magic145", "Quartaminorthirds", "Armodue",
    "Hirajoshi", "Scottish Bagpipes", "Thai Ranat",
    "Sevish 31-EDO", "11TET Machine", "13TET Father", "15TET Blackwood", "16TET Mavila", "16TET Mavila9", "17TET Superpyth",
    "22TET Orwell", "22TET Pajara", "22TET Pajara2", "22TET Porcupine", "26TET Flattone", "26TET Lemba", "46TET Sensi",
    "53TET Orwell", "72TET Prent", "Zeus Trivalent", "202TET Octone", "313TET Elfmadagasgar", "Marvel Glumma", "TOP Parapyth",
    "16ED", "15ED", "14ED", "13ED", "11ED", "10ED", "9ED", "8ED", "7ED", "6ED", "5ED",
    "16HD2", "15HD2", "14HD2", "13HD2", "12HD2", "11HD2", "10HD2", "9HD2", "8HD2", "7HD2", "6HD2", "5HD2",
    "32-16SD2", "30-15SD2", "28-14SD2", "26-13SD2", "24-12SD2", "22-11SD2", "20-10SD2", "18-9SD2", "16-8SD2", "14-7SD2", "12-6SD2", "10-5SD2", "8-4SD2",
    "BP Equal", "BP Just", "BP Lambda",
    "8-24HD3", "7-21HD3", "6-18HD3", "5-15HD3", "4-12HD3", "24-8HD3", "21-7HD3", "18-6HD3", "15-5HD3", "12-4HD3"
    // Add more names if you have more exotic scales, up to NUM_EXOTIC_SCALES
};

// --- Exotic scale intervals (static) ---
// --- Exotic scale intervals (static) ---
static const float exotic_scales[NUM_EXOTIC_SCALES][SCALE_MAX_LEN] = {


   // Blues major (From midipal/BitT source code)
    { 0.0f, 3.0f, 4.0f, 7.0f, 9.0f, 10.0f },
    // Blues minor (From midipal/BitT source code)
    { 0.0f, 3.0f, 5.0f, 6.0f, 7.0f, 10.0f },

    // Folk (From midipal/BitT source code)
    { 0.0f, 1.0f, 3.0f, 4.0f, 5.0f, 7.0f, 8.0f, 10.0f },
    // Japanese (From midipal/BitT source code)
    { 0.0f, 1.0f, 5.0f, 7.0f, 8.0f },
    // Gamelan (From midipal/BitT source code)
    { 0.0f, 1.0f, 3.0f, 7.0f, 8.0f },
    // Gypsy
    { 0.0f, 2.0f, 3.0f, 6.0f, 7.0f, 8.0f, 11.0f },
    // Arabian
    { 0.0f, 1.0f, 4.0f, 5.0f, 7.0f, 8.0f, 11.0f },
    // Flamenco
    { 0.0f, 1.0f, 4.0f, 5.0f, 7.0f, 8.0f, 10.0f },
    // Whole tone (From midipal/BitT source code)
    { 0.0f, 2.0f, 4.0f, 6.0f, 8.0f, 10.0f },
    // pythagorean (From yarns source code)
    { 0.0f, 0.898f, 2.039f, 2.938f, 4.078f, 4.977f, 6.117f, 7.023f, 7.922f, 9.062f, 9.961f, 11.102f },
    // 1_4_eb (From yarns source code)
    { 0.0f, 1.0f, 2.0f, 3.0f, 3.5f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f, 10.5f },
    // 1_4_e (From yarns source code)
    { 0.0f, 1.0f, 2.0f, 3.0f, 3.5f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f, 11.0f },
    // 1_4_ea (From yarns source code)
    { 0.0f, 1.0f, 2.0f, 3.0f, 3.5f, 5.0f, 6.0f, 7.0f, 8.0f, 8.5f, 10.0f, 11.0f },
    // bhairav (From yarns source code)
    { 0.0f, 0.898f, 3.859f, 4.977f, 7.023f, 7.922f, 10.883f },
    // gunakri (From yarns source code)
    { 0.0f, 1.117f, 4.977f, 7.023f, 8.141f },
    // marwa (From yarns source code)
    { 0.0f, 1.117f, 3.859f, 5.898f, 8.844f, 10.883f },
    // shree (From yarns source code)
    { 0.0f, 0.898f, 3.859f, 5.898f, 7.023f, 7.922f, 10.883f },
    // purvi (From yarns source code)
    { 0.0f, 1.117f, 3.859f, 5.898f, 7.023f, 8.141f, 10.883f },
    // bilawal (From yarns source code)
    { 0.0f, 2.039f, 3.859f, 4.977f, 7.023f, 9.062f, 10.883f },
    // yaman (From yarns source code)
    { 0.0f, 2.039f, 4.078f, 6.117f, 7.023f, 9.062f, 11.102f },
    // kafi (From yarns source code)
    { 0.0f, 1.820f, 2.938f, 4.977f, 7.023f, 8.844f, 9.961f },
    // bhimpalasree (From yarns source code)
    { 0.0f, 2.039f, 3.156f, 4.977f, 7.023f, 9.062f, 10.180f },
    // darbari (From yarns source code)
    { 0.0f, 2.039f, 2.938f, 4.977f, 7.023f, 7.922f, 9.961f },
    // rageshree (From yarns source code)
    { 0.0f, 2.039f, 3.859f, 4.977f, 7.023f, 8.844f, 9.961f },
    // khamaj (From yarns source code)
    { 0.0f, 2.039f, 3.859f, 4.977f, 7.023f, 9.062f, 9.961f, 11.102f },
    // mimal (From yarns source code)
    { 0.0f, 2.039f, 2.938f, 4.977f, 7.023f, 8.844f, 9.961f, 10.883f },
    // parameshwari (From yarns source code)
    { 0.0f, 0.898f, 2.938f, 4.977f, 8.844f, 9.961f },
    // rangeshwari (From yarns source code)
    { 0.0f, 2.039f, 2.938f, 4.977f, 7.023f, 10.883f },
    // gangeshwari (From yarns source code)
    { 0.0f, 3.859f, 4.977f, 7.023f, 7.922f, 9.961f },
    // kameshwari (From yarns source code)
    { 0.0f, 2.039f, 5.898f, 7.023f, 8.844f, 9.961f },
    // pa__kafi (From yarns source code)
    { 0.0f, 2.039f, 2.938f, 4.977f, 7.023f, 9.062f, 9.961f },
    // natbhairav (From yarns source code)
    { 0.0f, 2.039f, 3.859f, 4.977f, 7.023f, 7.922f, 10.883f },
    // m_kauns (From yarns source code)
    { 0.0f, 2.039f, 4.078f, 4.977f, 7.922f, 9.961f },
    // bairagi (From yarns source code)
    { 0.0f, 0.898f, 4.977f, 7.023f, 9.961f },
    // b_todi (From yarns source code)
    { 0.0f, 0.898f, 2.938f, 7.023f, 9.961f },
    // chandradeep (From yarns source code)
    { 0.0f, 2.938f, 4.977f, 7.023f, 9.961f },
    // kaushik_todi (From yarns source code)
    { 0.0f, 2.938f, 4.977f, 5.898f, 7.922f },
    // jogeshwari (From yarns source code)
    { 0.0f, 2.938f, 3.859f, 4.977f, 8.844f, 9.961f },

    // Tartini-Vallotti [12]
    { 0.0f, 0.9375f, 1.9609f, 2.9766f, 3.9219f, 5.0234f, 5.9219f, 6.9766f, 7.9609f, 8.9375f, 10.0f, 10.8984f },
    // 13 out of 22-tET, generator = 5 [13]
    { 0.0f, 1.0938f, 2.1797f, 3.2734f, 3.8203f, 4.9063f, 6.0f, 6.5469f, 7.6328f, 8.7266f, 9.2734f, 10.3672f, 11.4531f },
    // 13 out of 19-tET, Mandelbaum [13]
    { 0.0f, 1.2656f, 1.8984f, 3.1563f, 3.7891f, 5.0547f, 5.6875f, 6.9453f, 7.5781f, 8.8438f, 9.4766f, 10.7344f, 11.3672f },
    // Magic[16] in 145-tET [16]
    { 0.0f, 1.4922f, 2.0703f, 2.6484f, 3.2266f, 3.8047f, 4.3828f, 5.8750f, 6.4531f, 7.0313f, 7.6172f, 8.1953f, 9.6797f, 10.2656f, 10.8438f, 11.4219f },
    // g=9 steps of 139-tET. Gene Ward Smith "Quartaminorthirds" 7-limit temperament [16]
    { 0.0f, 0.7734f, 1.5547f, 2.3281f, 3.1094f, 3.8828f, 4.6641f, 5.4375f, 6.2188f, 6.9922f, 7.7734f, 8.5469f, 9.3203f, 10.1016f, 10.8750f, 11.6563f },
    // Armodue semi-equalizzato [16]
    { 0.0f, 0.7734f, 1.5469f, 2.3203f, 3.0938f, 3.8672f, 4.6484f, 5.4219f, 6.1953f, 6.9688f, 7.7422f, 8.5156f, 9.2891f, 9.6797f, 10.4531f, 11.2266f },

    // Hirajoshi[5]
    { 0.0f, 1.8516f, 3.3672f, 6.8281f, 7.8984f },
    // Scottish bagpipes[7]
    { 0.0f, 1.9688f, 3.4063f, 4.9531f, 7.0313f, 8.5313f, 10.0938f },
    // Thai ranat[7]
    { 0.0f, 1.6094f, 3.4609f, 5.2578f, 6.8594f, 8.6172f, 10.2891f },

    // Sevish quasi-12-equal mode from 31-EDO
    { 0.0f, 1.1641f, 2.3203f, 3.0938f, 4.2578f, 5.0313f, 6.1953f, 7.3516f, 8.1328f, 9.2891f, 10.0625f, 11.2266f },
    // 11 TET Machine[6]
    { 0.0f, 2.1797f, 4.3672f, 5.4531f, 7.6328f, 9.8203f },
    // 13 TET Father[8]
    { 0.0f, 1.8438f, 3.6953f, 4.6172f, 6.4609f, 8.3047f, 9.2344f, 11.0781f },
    // 15 TET Blackwood[10]
    { 0.0f, 1.6016f, 2.3984f, 4.0f, 4.7969f, 6.3984f, 7.2031f, 8.7969f, 9.6016f, 11.2031f },
    // 16 TET Mavila[7]
    { 0.0f, 1.5f, 3.0f, 5.25f, 6.75f, 8.25f, 9.75f },
    // 16 TET Mavila[9]
    { 0.0f, 0.75f, 2.25f, 3.75f, 5.25f, 6.0f, 7.5f, 9.0f, 10.5f },
    // 17 TET Superpyth[12]
    { 0.0f, 0.7031f, 1.4141f, 2.8203f, 3.5313f, 4.9375f, 5.6484f, 6.3516f, 7.7578f, 8.4688f, 9.8828f, 10.5859f },

    // 22 TET Orwell[9]
    { 0.0f, 1.0938f, 2.7266f, 3.8203f, 5.4531f, 6.5469f, 8.1797f, 9.2734f, 10.9063f },
    // 22 TET Pajara[10] Static Symmetrical Maj
    { 0.0f, 1.0938f, 2.1797f, 3.8203f, 4.9063f, 6.0f, 7.0938f, 8.1797f, 9.8203f, 10.9063f },
    // 22 TET Pajara[10] Std Pentachordal Maj
    { 0.0f, 1.0938f, 2.1797f, 3.8203f, 4.9063f, 6.0f, 7.0938f, 8.7266f, 9.8203f, 10.9063f },
    // 22 TET Porcupine[7]
    { 0.0f, 1.6328f, 3.2734f, 4.9063f, 7.0938f, 8.7266f, 10.3672f },
    // 26 TET Flattone[12]
    { 0.0f, 0.4609f, 1.8438f, 2.3047f, 3.6953f, 5.0781f, 5.5391f, 6.9219f, 7.3828f, 8.7656f, 9.2266f, 10.6172f },
    // 26 TET Lemba[10]
    { 0.0f, 1.3828f, 2.3047f, 3.6953f, 4.6172f, 6.0f, 7.3828f, 8.3047f, 9.6875f, 10.6172f },
    // 46 TET Sensi[11]
    { 0.0f, 1.3047f, 2.6094f, 3.9141f, 4.4375f, 5.7422f, 7.0469f, 8.3516f, 8.8672f, 10.1719f, 11.4766f },
    // 53 TET Orwell[9]
    { 0.0f, 1.1328f, 2.7188f, 3.8516f, 5.4375f, 6.5625f, 8.1484f, 9.2813f, 10.8672f },
    // 12 out of 72-TET scale by Prent Rodgers
    { 0.0f, 2.0f, 2.6641f, 3.8359f, 4.3359f, 5.0f, 5.5f, 7.0f, 8.8359f, 9.6641f, 10.5f, 10.8359f },
    // Trivalent scale in zeus temperament[7]
    { 0.0f, 1.5781f, 3.8750f, 5.4531f, 7.0313f, 9.3359f, 10.9063f },
    // 202 TET tempering of octone[8]
    { 0.0f, 1.1875f, 3.5078f, 3.8594f, 6.1797f, 7.0078f, 9.3281f, 9.6797f },
    // 313 TET elfmadagasgar[9]
    { 0.0f, 2.0313f, 2.4922f, 4.5234f, 4.9844f, 7.0156f, 7.4766f, 9.5078f, 9.9688f },
    // Marvel woo version of glumma[12]
    { 0.0f, 0.4922f, 2.3281f, 3.1719f, 3.8359f, 5.4922f, 6.1641f, 7.0078f, 8.8359f, 9.3281f, 9.6797f, 11.6563f },
    // TOP Parapyth[12]
    { 0.0f, 0.5859f, 2.0703f, 2.6563f, 4.1406f, 4.7266f, 5.5469f, 7.0469f, 7.6172f, 9.1094f, 9.6875f, 11.1797f },

    // 16-ED (ED2 or ED3)
    { 0.0f, 0.75f, 1.5f, 2.25f, 3.0f, 3.75f, 4.5f, 5.25f, 6.0f, 6.75f, 7.5f, 8.25f, 9.0f, 9.75f, 10.5f, 11.25f },
    // 15-ED (ED2 or ED3)
    { 0.0f, 0.7969f, 1.6016f, 2.3984f, 3.2031f, 4.0f, 4.7969f, 5.6016f, 6.3984f, 7.2031f, 8.0f, 8.7969f, 9.6016f, 10.3984f, 11.2031f },
    // 14-ED (ED2 or ED3)
    { 0.0f, 0.8594f, 1.7109f, 2.5703f, 3.4297f, 4.2891f, 5.1484f, 6.0f, 6.8594f, 7.7188f, 8.5781f, 9.4375f, 10.2969f, 11.1563f },
    // 13-ED (ED2 or ED3)
    { 0.0f, 0.9219f, 1.8438f, 2.7656f, 3.6953f, 4.6328f, 5.6328f, 6.5703f, 7.4922f, 8.4141f, 9.3359f, 10.2578f, 11.1797f },
    // 11-ED (ED2 or ED3)
    { 0.0f, 1.0938f, 2.1797f, 3.2734f, 4.3672f, 5.4531f, 6.5469f, 7.6328f, 8.7266f, 9.8203f, 10.9063f },
    // 10-ED (ED2 or ED3)
    { 0.0f, 1.2031f, 2.3984f, 3.6016f, 4.7969f, 6.0f, 7.2031f, 8.3984f, 9.6016f, 10.7969f },
    // 9-ED (ED2 or ED3)
    { 0.0f, 1.3359f, 2.6641f, 4.0f, 5.3359f, 6.6641f, 8.0f, 9.3359f, 10.6641f },
    // 8-ED (ED2 or ED3)
    { 0.0f, 1.5f, 3.0f, 4.5f, 6.0f, 7.5f, 9.0f, 10.5f },
    // 7-ED (ED2 or ED3)
    { 0.0f, 1.7109f, 3.4297f, 5.1484f, 6.8594f, 8.5781f, 10.2969f },
    // 6-ED (ED2 or ED3)
    { 0.0f, 2.0f, 4.0f, 6.0f, 8.0f, 10.0f },
    // 5-ED (ED2 or ED3)
    { 0.0f, 2.3984f, 4.7969f, 7.2031f, 9.6016f },

    // 16-HD2 (16 step harmonic series scale on the octave)
    { 0.0f, 1.0469f, 2.0391f, 2.9766f, 3.8594f, 4.7109f, 5.5156f, 6.2813f, 7.0234f, 7.7266f, 8.4063f, 9.0625f, 9.6875f, 10.2969f, 10.8906f, 11.4531f },
    // 15-HD2 (15 step harmonic series scale on the octave)
    { 0.0f, 1.1172f, 2.1641f, 3.1563f, 4.0938f, 4.9766f, 5.8203f, 6.6328f, 7.4141f, 8.1641f, 8.8828f, 9.5703f, 10.2266f, 10.852f, 11.4453f },
    // 14-HD2 (14 step harmonic series scale on the octave)
    { 0.0f, 1.1953f, 2.3125f, 3.3594f, 4.3516f, 5.2891f, 6.1797f, 7.0313f, 7.8516f, 8.6406f, 9.3984f, 10.125f, 10.8203f, 11.4844f },
    // 13-HD2 (13 step harmonic series scale on the octave)
    { 0.0f, 1.2813f, 2.4766f, 3.5938f, 4.6406f, 5.6328f, 6.5703f, 7.4609f, 8.3125f, 9.125f, 9.9063f, 10.6484f, 11.3594f },
    // 12-HD2 (12 step harmonic series scale on the octave)
    { 0.0f, 1.3828f, 2.6719f, 3.8594f, 5.0078f, 6.0313f, 6.9922f, 7.9531f, 8.8438f, 9.6875f, 10.4844f, 11.2656f },
    // 11-HD2 (11 step harmonic series scale on the octave)
    { 0.0f, 1.5078f, 2.8906f, 4.1719f, 5.3672f, 6.4844f, 7.5391f, 8.5234f, 9.4688f, 10.3672f, 11.2109f },
    // 10-HD2 (10 step harmonic series scale on the octave)
    { 0.0f, 1.6484f, 3.1563f, 4.5391f, 5.8672f, 7.0234f, 8.0703f, 9.1875f, 10.1797f, 11.1094f },
    // 9-HD2 (9 step harmonic series scale on the octave)
    { 0.0f, 1.8203f, 3.4766f, 5.0938f, 6.6797f, 8.2422f, 9.7891f, 11.3203f, 12.0f },
    // 8-HD2 (8 step harmonic series scale on the octave)
    { 0.0f, 2.0391f, 3.8594f, 5.5156f, 7.0234f, 8.4063f, 9.6875f, 10.8906f },
    // 7-HD2 (7 step harmonic series scale on the octave)
    { 0.0f, 2.3125f, 4.3516f, 6.1797f, 7.8516f, 9.3984f, 10.8203f },
    // 6-HD2 (6 step harmonic series scale on the octave)
    { 0.0f, 3.0313f, 6.0313f, 9.0625f, 12.0f, 15.0f },
    // 5-HD2 (5 step harmonic series scale on the octave)
    { 0.0f, 4.0f, 8.0f, 12.0f, 16.0f },

    // 32-16-SD2 (16 step subharmonic series scale on the octave)
    { 0.0f, 0.5469f, 1.1172f, 1.7031f, 2.3125f, 2.9375f, 3.5938f, 4.2734f, 4.9766f, 5.7188f, 6.4844f, 7.2891f, 8.0234f, 8.9297f, 9.9609f, 10.9531f },
    // 30-15-SD2 (15 step subharmonic series scale on the octave)
    { 0.0f, 0.5859f, 1.1953f, 1.8203f, 2.4766f, 3.1563f, 3.8594f, 4.6016f, 5.3672f, 6.1797f, 7.0313f, 7.9063f, 8.8438f, 9.8359f, 10.8828f },
    // 28-14-SD2 (14 step subharmonic series scale on the octave)
    { 0.0f, 0.6328f, 1.2813f, 1.9609f, 2.6719f, 3.4063f, 4.1719f, 4.977f, 5.8203f, 6.6953f, 7.6328f, 8.6328f, 9.6875f, 10.8047f, 12.0f },
    // 26-13-SD2 (13 step subharmonic series scale on the octave)
    { 0.0f, 0.6797f, 1.3828f, 2.125f, 2.8906f, 3.6953f, 4.5391f, 5.4219f, 6.3516f, 7.3203f, 8.3281f, 9.375f, 10.4609f },
    // 24-12-SD2 (12 step subharmonic series scale on the octave)
    { 0.0f, 0.7344f, 1.5078f, 2.3125f, 3.1563f, 4.0469f, 4.9766f, 5.9531f, 6.9688f, 8.0234f, 9.1172f, 10.25f },
    // 22-11-SD2 (11 step subharmonic series scale on the octave)
    { 0.0f, 0.8047f, 1.6484f, 2.5391f, 3.4766f, 4.4609f, 5.4922f, 6.5703f, 7.6953f, 8.8672f, 10.0859f },
    // 20-10-SD2 (10 step subharmonic series scale on the octave)
    { 0.0f, 0.8906f, 1.8203f, 2.8125f, 3.8594f, 4.9609f, 6.1172f, 7.3281f, 8.5938f, 9.9141f },
    // 18-9-SD2 (9 step subharmonic series scale on the octave)
    { 0.0f, 0.9922f, 2.0391f, 3.1563f, 4.3359f, 5.5781f, 6.8828f, 8.25f, 9.6797f },
    // 16-8-SD2 (8 step subharmonic series scale on the octave)
    { 0.0f, 1.1172f, 2.3125f, 3.5938f, 4.9609f, 6.4141f, 7.9531f, 9.5781f },
    // 14-7-SD2 (7 step subharmonic series scale on the octave)
    { 0.0f, 1.2813f, 2.6719f, 4.1719f, 5.7891f, 7.5234f, 9.375f },
    // 12-6-SD2 (6 step subharmonic series scale on the octave)
    { 0.0f, 1.5078f, 3.1563f, 4.9609f, 6.9219f, 9.0391f },
    // 10-5-SD2 (5 step subharmonic series scale on the octave)
    { 0.0f, 1.8203f, 3.8594f, 6.1719f, 8.8438f },
    // 8-4-SD2 (4 step subharmonic series scale on the octave)
    { 0.0f, 2.3125f, 4.9766f, 8.1406f },

    // Bohlen-Pierce (equal)
    { 0.0f, 0.9219f, 1.8438f, 2.7656f, 3.6953f, 4.6172f, 5.5391f, 6.4609f, 7.3828f, 8.3047f, 9.2344f, 10.1563f, 11.0781f },
    // Bohlen-Pierce (just)
    { 0.0f, 0.8438f, 1.9063f, 2.7422f, 3.6719f, 4.6484f, 5.5781f, 6.4219f, 7.3516f, 8.3281f, 9.2578f, 10.0938f, 11.1563f },
    // Bohlen-Pierce (lambda)
    { 0.0f, 1.9063f, 2.7422f, 3.6719f, 5.5781f, 6.4219f, 8.3281f, 9.2578f, 11.1563f },

    // 8-24-HD3 (16 step harmonic series scale on the tritave)
    { 0.0f, 1.2891f, 2.4375f, 3.4766f, 4.4297f, 5.3047f, 6.1172f, 6.8828f, 7.6172f, 8.3203f, 9.0f, 9.6641f, 10.3125f, 10.9453f, 11.5625f, 12.1563f },
    // 7-21-HD3 (14 step harmonic series scale on the tritave)
    { 0.0f, 1.4609f, 2.7422f, 3.8984f, 4.9375f, 5.8672f, 6.6953f, 7.4297f, 8.0781f, 8.6484f, 9.1484f, 9.5859f, 9.9688f, 10.3047f },
    // 6-18-HD3 (12 step harmonic series scale on the tritave)
    { 0.0f, 1.6875f, 3.1406f, 4.4297f, 5.5703f, 6.5703f, 7.4375f, 8.1797f, 8.8047f, 9.3203f, 9.7344f, 10.0547f },
    // 5-15-HD3 (10 step harmonic series scale on the tritave)
    { 0.0f, 1.9922f, 3.6719f, 5.1328f, 6.3828f, 7.4297f, 8.2813f, 8.9453f, 9.4297f, 9.7422f },
    // 4-12-HD3 (8 step harmonic series scale on the tritave)
    { 0.0f, 2.4375f, 4.4297f, 6.1172f, 7.6172f, 9.0f, 10.3125f, 11.5625f },

    // 24-8-HD3 (16 step subharmonic series scale on the tritave)
    { 0.0f, 0.4688f, 0.9531f, 1.4609f, 1.9922f, 2.5469f, 3.125f, 3.7266f, 4.3516f, 5.0f, 5.6719f, 6.3672f, 7.0859f, 7.8281f, 8.5938f, 9.3828f },
    // 21-7-HD3 (14 step subharmonic series scale on the tritave)
    { 0.0f, 0.5313f, 1.0938f, 1.6875f, 2.3047f, 2.9453f, 3.6094f, 4.2969f, 5.0078f, 5.7422f, 6.5f, 7.2813f, 8.0859f, 8.9141f },
    // 18-6-HD3 (12 step subharmonic series scale on the tritave)
    { 0.0f, 0.625f, 1.2891f, 1.9922f, 2.7344f, 3.5156f, 4.3359f, 5.1953f, 6.0938f, 7.0313f, 8.0078f, 9.0234f },
    // 15-5-HD3 (10 step subharmonic series scale on the tritave)
    { 0.0f, 0.75f, 1.5625f, 2.4375f, 3.375f, 4.375f, 5.4375f, 6.5625f, 7.75f, 9.0f },
    // 12-4-HD3 (8 step subharmonic series scale on the tritave)
    { 0.0f, 0.9531f, 1.9922f, 3.125f, 4.3516f, 5.6719f, 7.0859f, 8.5938f }
};

// --- Standard scale intervals (algorithmic) ---
void get_standard_scale_intervals(int scaleIdx, int* out, int* outLen) {
    switch (scaleIdx) {
        case 0: out[0]=0; out[1]=2; out[2]=4; out[3]=5; out[4]=7; out[5]=9; out[6]=11; out[7]=12; *outLen=8; break; // Major
        case 1: out[0]=0; out[1]=2; out[2]=3; out[3]=5; out[4]=7; out[5]=8; out[6]=10; out[7]=12; *outLen=8; break; // Minor
        case 2: out[0]=0; out[1]=2; out[2]=3; out[3]=5; out[4]=7; out[5]=8; out[6]=11; out[7]=12; *outLen=8; break; // Harmonic Minor
        case 3: out[0]=0; out[1]=2; out[2]=3; out[3]=5; out[4]=7; out[5]=9; out[6]=11; out[7]=12; *outLen=8; break; // Melodic Minor
        case 4: out[0]=0; out[1]=2; out[2]=4; out[3]=5; out[4]=7; out[5]=9; out[6]=10; out[7]=12; *outLen=8; break; // Mixolydian
        case 5: out[0]=0; out[1]=2; out[2]=3; out[3]=5; out[4]=7; out[5]=9; out[6]=10; out[7]=12; *outLen=8; break; // Dorian
        case 6: out[0]=0; out[1]=2; out[2]=4; out[3]=6; out[4]=7; out[5]=9; out[6]=11; out[7]=12; *outLen=8; break; // Lydian
        case 7: out[0]=0; out[1]=1; out[2]=3; out[3]=5; out[4]=7; out[5]=8; out[6]=10; out[7]=12; *outLen=8; break; // Phrygian
        case 8: out[0]=0; out[1]=2; out[2]=3; out[3]=5; out[4]=7; out[5]=8; out[6]=10; out[7]=12; *outLen=8; break; // Aeolian
        case 9: out[0]=0; out[1]=1; out[2]=3; out[3]=5; out[4]=6; out[5]=8; out[6]=10; out[7]=12; *outLen=8; break; // Locrian
        case 10: out[0]=0; out[1]=2; out[2]=4; out[3]=7; out[4]=9; out[5]=12; out[6]=0; out[7]=0; *outLen=5; break; // Maj Pent
        case 11: out[0]=0; out[1]=3; out[2]=5; out[3]=7; out[4]=10; out[5]=12; out[6]=0; out[7]=0; *outLen=5; break; // Min Pent
        case 12: out[0]=0; out[1]=2; out[2]=4; out[3]=6; out[4]=8; out[5]=10; out[6]=12; out[7]=0; *outLen=7; break; // Whole Tone
        case 13: out[0]=0; out[1]=1; out[2]=3; out[3]=4; out[4]=6; out[5]=7; out[6]=9; out[7]=10; *outLen=8; break; // Octatonic HW
        case 14: out[0]=0; out[1]=2; out[2]=3; out[3]=5; out[4]=6; out[5]=8; out[6]=9; out[7]=11; *outLen=8; break; // Octatonic WH
        case 15: out[0]=0; out[1]=2; out[2]=4; out[3]=5; out[4]=7; out[5]=9; out[6]=11; out[7]=12; *outLen=8; break; // Ionian
        default: for (int i = 0; i < SCALE_MAX_LEN; ++i) out[i] = 0; *outLen = 0;
    }
}

// --- ByteBeat equations (Viznutcracker, sweet! and others) ---
#define NUM_BYTEBEAT_EQNS 16
static const char* bytebeat_names[NUM_BYTEBEAT_EQNS] = {
    "hope", "love", "life", "age", "clysm", "monk", "NERV", "Trurl",
    "Pirx", "Snaut", "Hari", "Kris", "Tichy", "Bregg", "Avon", "Orac"
};

static const char* bytebeat_cv1_dest_names[] = {
    "igain", "eqn", "P0", "P1", "P2"
};
#define NUM_BYTEBEAT_CV1_DEST 5

float bytebeat(int eqn, int t, int p0, int p1, int p2) {
    switch (eqn) {
        case 0: return ((t * (t >> 8)) & 0xFF) / 128.0f - 1.0f; // hope
        case 1: return ((t ^ (t >> 3)) & 0xFF) / 128.0f - 1.0f; // love
        case 2: return ((t * ((t >> 5) | (t >> 8))) & 0xFF) / 128.0f - 1.0f; // life
        case 3: return ((t * 42 & (t >> 10)) & 0xFF) / 128.0f - 1.0f; // age
        case 4: return ((((t * 9) & (t >> 4)) | ((t * 5) & (t >> 7))) & 0xFF) / 128.0f - 1.0f; // clysm
        case 5: return ((((t * 5) & (t >> 7)) | ((t * 3) & (t >> 10))) & 0xFF) / 128.0f - 1.0f; // monk
        case 6: return ((t * 7 & (t >> 11)) & 0xFF) / 128.0f - 1.0f; // NERV
        case 7: return ((t * 13 & (t >> 8)) & 0xFF) / 128.0f - 1.0f; // Trurl
        case 8: return ((t * (t >> 6 | t >> 8)) & 0xFF) / 128.0f - 1.0f; // Pirx
        case 9: return ((t ^ (t >> 5)) & 0xFF) / 128.0f - 1.0f; // Snaut
        case 10: return ((t * 11 & (t >> 9)) & 0xFF) / 128.0f - 1.0f; // Hari
        case 11: return ((t * 17 & (t >> 7)) & 0xFF) / 128.0f - 1.0f; // Kris
        case 12: return ((t * 19 & (t >> 6)) & 0xFF) / 128.0f - 1.0f; // Tichy
        case 13: return ((t * 23 & (t >> 5)) & 0xFF) / 128.0f - 1.0f; // Bregg
        case 14: return ((t * 29 & (t >> 4)) & 0xFF) / 128.0f - 1.0f; // Avon
        case 15: return ((t * 31 & (t >> 3)) & 0xFF) / 128.0f - 1.0f; // Orac
        default: return 0.0f;
    }
}

// --- Integer Sequence state ---
struct IntSeqState {
    int pos;
    int dir; // 1 = forward, -1 = backward
};

// --- State for the algorithm ---
struct CopierMaschineState {
    float buffer[ASR_BUF_SIZE]; // Circular buffer for ASR
    int bufLen;                 // Current buffer length
    int writePos;               // Current write position in buffer
    float lastClock;            // Last clock value for edge detection
    int t;                      // ByteBeat time counter
    IntSeqState intseq;         // Integer Sequence state
};

// --- Algorithm struct ---
struct _copierAlgorithm : public _NT_algorithm {
    CopierMaschineState* state;
};

// --- Parameter enum ---
enum {
    kParamInputCV,
    kParamClock,
    kParamOutputA,
    kParamOutputB,
    kParamOutputC,
    kParamOutputD,
    kParamScale,        // 0..NUM_SCALES-1
    kParamRoot,         // 0..11 (C..B)
    kParamTranspose,    // -24..+24 semitones
    kParamMaskRotate,   // 0..15
    kParamBufIndex,     // 0..(ASR_BUF_SIZE-1)
    kParamBufLen,       // 4..ASR_BUF_SIZE
    kParamHold,         // 0=off, 1=on
    kParamGain,         // 5..200 (scaled by 0.01)
    kParamCVSource,     // 0=CV, 1=ByteBeat, 2=IntSeq
    kParamByteBeatEqn,  // 0..NUM_BYTEBEAT_EQNS-1
    kParamByteBeatP0,   // 0..255
    kParamByteBeatP1,   // 0..255
    kParamByteBeatP2,   // 0..255
    kParamByteBeatCV1Dest, // 0..4
    // Integer Sequence params:
    kParamIntSeq,       // 0..NUM_INTSEQ-1
    kParamIntSeqMod,    // 1..32 (modulus)
    kParamIntSeqStart,  // 0..126
    kParamIntSeqLen,    // 2..128
    kParamIntSeqDir,    // 0=loop, 1=pendulum
    kParamIntSeqStride, // 1..16
    kParamIntSeqCV1Dest,// 0..NUM_INTSEQ_CV1_DEST-1
    kNumParams
};

// --- Parameter definitions ---
static const _NT_parameter parameters[] = {
    { .name = "CV In", .min = 1, .max = 28, .def = 1, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
    { .name = "Clock", .min = 1, .max = 28, .def = 2, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
    { .name = "Out A", .min = 1, .max = 28, .def = 13, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
    { .name = "Out B", .min = 1, .max = 28, .def = 14, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
    { .name = "Out C", .min = 1, .max = 28, .def = 15, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
    { .name = "Out D", .min = 1, .max = 28, .def = 16, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
    { .name = "Scale", .min = 0, .max = NUM_SCALES-1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = all_scale_names },
    { .name = "Root", .min = 0, .max = 11, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
    { .name = "Transpose", .min = -24, .max = 24, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
    { .name = "MaskRot", .min = 0, .max = 15, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
    { .name = "BufIdx", .min = 0, .max = ASR_BUF_SIZE-1, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
    { .name = "BufLen", .min = 4, .max = ASR_BUF_SIZE, .def = 16, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
    { .name = "Hold", .min = 0, .max = 1, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
    { .name = "Gain", .min = 5, .max = 200, .def = 100, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
    { .name = "CVSrc", .min = 0, .max = 2, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = (const char*[]){"CV", "ByteBeat", "IntSeq"} },
    { .name = "BB Eqn", .min = 0, .max = NUM_BYTEBEAT_EQNS-1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = bytebeat_names },
    { .name = "BB P0", .min = 0, .max = 255, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
    { .name = "BB P1", .min = 0, .max = 255, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
    { .name = "BB P2", .min = 0, .max = 255, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
    { .name = "BB CV1", .min = 0, .max = NUM_BYTEBEAT_CV1_DEST-1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = bytebeat_cv1_dest_names },
    { .name = "IntSeq", .min = 0, .max = NUM_INTSEQ-1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = intseq_names },
    { .name = "IntSeqMod", .min = 1, .max = 32, .def = 8, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
    { .name = "IntSeqStart", .min = 0, .max = 126, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
    { .name = "IntSeqLen", .min = 2, .max = 128, .def = 16, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
    { .name = "IntSeqDir", .min = 0, .max = 1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = (const char*[]){"loop", "pendulum"} },
    { .name = "IntSeqStride", .min = 1, .max = 16, .def = 1, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
    { .name = "IntSeqCV1", .min = 0, .max = NUM_INTSEQ_CV1_DEST-1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = intseq_cv1_dest_names },
};

// --- Algorithm requirements ---
void calculateRequirements(_NT_algorithmRequirements& req, const int32_t*) {
    req.numParameters = kNumParams;
    req.sram = sizeof(_copierAlgorithm);
    req.dram = sizeof(CopierMaschineState);
    req.dtc = 0;
    req.itc = 0;
}

// --- Algorithm construction ---
_NT_algorithm* construct(const _NT_algorithmMemoryPtrs& ptrs, const _NT_algorithmRequirements&, const int32_t*) {
    _copierAlgorithm* alg = reinterpret_cast<_copierAlgorithm*>(ptrs.sram);
    alg->state = reinterpret_cast<CopierMaschineState*>(ptrs.dram);
    memset(alg->state, 0, sizeof(CopierMaschineState));
    alg->parameters = parameters;
    alg->parameterPages = NULL;
    return alg;
}

// --- Quantization function ---
float quantize(float v, int scaleIdx, int root, int transpose, int maskRotate) {
    int scale[SCALE_MAX_LEN];
    int scaleLen = 0;
    if (scaleIdx < NUM_STANDARD_SCALES) {
        get_standard_scale_intervals(scaleIdx, scale, &scaleLen);
    } else if (scaleIdx < NUM_STANDARD_SCALES + NUM_EXOTIC_SCALES) {
        int exoticIdx = scaleIdx - NUM_STANDARD_SCALES;
        for (int i = 0; i < SCALE_MAX_LEN; ++i) scale[i] = (int)exotic_scales[exoticIdx][i];
        scaleLen = SCALE_MAX_LEN;
    } else {
        for (int i = 0; i < SCALE_MAX_LEN; ++i) scale[i] = 0;
        scaleLen = 0;
    }
    float note = v * 12.0f;
    int n = static_cast<int>(roundf(note));
    n += root + transpose;
    int scaleDegree = 0;
    int minDist = 128;
    for (int i = 0; i < scaleLen; ++i) {
        int deg = (scale[i] + maskRotate) % 12;
        int dist = abs((n % 12) - deg);
        if (dist < minDist) {
            minDist = dist;
            scaleDegree = i;
        }
    }
    int quantized = (n / 12) * 12 + scale[scaleDegree];
    return quantized / 12.0f;
}

// --- Integer Sequence stepping function ---
int intseq_step(IntSeqState& state, int seqIdx, int start, int len, int stride, int dirMode) {
    if (dirMode == 1) {
        if (state.dir == 1 && state.pos >= len-1) state.dir = -1;
        else if (state.dir == -1 && state.pos <= 0) state.dir = 1;
    }
    int idx = start + state.pos;
    if (idx < 0) idx = 0;
    if (idx >= INTSEQ_MAX_LEN) idx = INTSEQ_MAX_LEN-1;
    int value = intseq_tables[seqIdx][idx];
    state.pos += stride * state.dir;
    if (dirMode == 0) {
        if (state.pos >= len) state.pos = 0;
        if (state.pos < 0) state.pos = len-1;
    }
    return value;
}

// --- Main processing loop ---
void step(_NT_algorithm* self, float* busFrames, int numFramesBy4) {
    _copierAlgorithm* alg = (_copierAlgorithm*)self;
    CopierMaschineState* state = alg->state;
    int numFrames = numFramesBy4 * 4;

    int inCV_idx = alg->v[kParamInputCV] - 1;
    int clock_idx = alg->v[kParamClock] - 1;
    int outA_idx = alg->v[kParamOutputA] - 1;
    int outB_idx = alg->v[kParamOutputB] - 1;
    int outC_idx = alg->v[kParamOutputC] - 1;
    int outD_idx = alg->v[kParamOutputD] - 1;

    float* inCV = busFrames + inCV_idx * numFrames;
    float* clock = busFrames + clock_idx * numFrames;
    float* outA = busFrames + outA_idx * numFrames;
    float* outB = busFrames + outB_idx * numFrames;
    float* outC = busFrames + outC_idx * numFrames;
    float* outD = busFrames + outD_idx * numFrames;

    int scale = alg->v[kParamScale];
    int root = alg->v[kParamRoot];
    int transpose = alg->v[kParamTranspose];
    int maskRotate = alg->v[kParamMaskRotate];
    int bufIdx = alg->v[kParamBufIndex];
    int bufLen = alg->v[kParamBufLen];
    if (bufLen < 4) bufLen = 4;
    if (bufLen > ASR_BUF_SIZE) bufLen = ASR_BUF_SIZE;
    state->bufLen = bufLen;
    bool hold = alg->v[kParamHold] != 0;
    float gain = alg->v[kParamGain] * 0.01f;
    int cvSource = alg->v[kParamCVSource];
    int bbEqn = alg->v[kParamByteBeatEqn];
    int bbP0 = alg->v[kParamByteBeatP0];
    int bbP1 = alg->v[kParamByteBeatP1];
    int bbP2 = alg->v[kParamByteBeatP2];

    int intSeqIdx = alg->v[kParamIntSeq];
    int intSeqMod = alg->v[kParamIntSeqMod];
    int intSeqStart = alg->v[kParamIntSeqStart];
    int intSeqLen = alg->v[kParamIntSeqLen];
    int intSeqDir = alg->v[kParamIntSeqDir];
    int intSeqStride = alg->v[kParamIntSeqStride];

    // Initialize IntSeq state if needed
    if (cvSource == 2) {
        if (state->intseq.dir == 0) state->intseq.dir = 1;
        if (state->intseq.pos < 0 || state->intseq.pos >= intSeqLen) state->intseq.pos = 0;
    }

    for (int i = 0; i < numFrames; ++i) {
        bool clk = (clock[i] > 1.0f && state->lastClock <= 1.0f);
        state->lastClock = clock[i];

        float sample = 0.0f;
        if (cvSource == 0) {
            sample = inCV[i] * gain;
        } else if (cvSource == 1) {
            sample = bytebeat(bbEqn, state->t++, bbP0, bbP1, bbP2);
        } else if (cvSource == 2) {
            int val = intseq_step(state->intseq, intSeqIdx, intSeqStart, intSeqLen, intSeqStride, intSeqDir);
            sample = (val % intSeqMod) / 12.0f;
        }

        if (clk && !hold) {
            state->buffer[state->writePos] = sample;
            state->writePos = (state->writePos + 1) % state->bufLen;
        }

        int base = state->writePos;
        int idxA = (base - 1 - bufIdx * 1 + state->bufLen) % state->bufLen;
        int idxB = (base - 1 - bufIdx * 2 + state->bufLen) % state->bufLen;
        int idxC = (base - 1 - bufIdx * 3 + state->bufLen) % state->bufLen;
        int idxD = (base - 1 - bufIdx * 4 + state->bufLen) % state->bufLen;

        float qA = quantize(state->buffer[idxA], scale, root, transpose, maskRotate);
        float qB = quantize(state->buffer[idxB], scale, root, transpose, maskRotate);
        float qC = quantize(state->buffer[idxC], scale, root, transpose, maskRotate);
        float qD = quantize(state->buffer[idxD], scale, root, transpose, maskRotate);

        outA[i] = qA;
        outB[i] = qB;
        outC[i] = qC;
        outD[i] = qD;
    }
}

// --- Factory definition ---
static const _NT_factory factory = {
    .guid = NT_MULTICHAR('C','P','M','T'),
    .name = "CopierMaschine",
    .description = "Quantizing ASR with a lot of scales, Viznutcracker ByteBeat, Integer Sequences, and scale selection",
    .numSpecifications = 0,
    .calculateRequirements = calculateRequirements,
    .construct = construct,
    .parameterChanged = NULL,
    .step = step,
    .draw = NULL,
    .midiMessage = NULL,
};

// --- Plugin entry point ---
uintptr_t pluginEntry(_NT_selector selector, uint32_t data) {
    switch (selector) {
        case kNT_selector_version: return kNT_apiVersionCurrent;
        case kNT_selector_numFactories: return 1;
        case kNT_selector_factoryInfo: return (uintptr_t)((data == 0) ? &factory : NULL);
    }
    return 0;
}