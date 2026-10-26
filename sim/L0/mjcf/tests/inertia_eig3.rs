//! A full inertia's principal moments and frame, bit for bit MuJoCo 3.5.0's;
//! the inertias MuJoCo refuses; and a non-finite inertia refused, not a hang.
//!
//! `CASES` are MuJoCo 3.5.0's `body_inertia` and `body_iquat`, as bits, for a
//! free body whose `<inertial fullinertia>` is each tensor, measured on the
//! census's unfused oracle: two equal moments with an off-diagonal below the
//! iteration's stop, six diagonal permutations, `2 2 2 0.5 0 0`, three
//! axisymmetric tensors, forty box moments turned by seeded random rotations
//! (written as Python's `repr`, the shortest text that reads back to the same
//! double, so both sides read the same bits) and three tensors whose largest
//! off-diagonals tie, where the pivot's strict comparison picks the frame.
//! MuJoCo's compiler finds them with `mjuu_eig3` (`user_util.cc:661-755`)
//! through `mjuu_fullInertia` (`:785-814`).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
// The cases are bit patterns, written as MuJoCo's values' raw bits.
#![allow(clippy::unreadable_literal)]

use sim_mjcf::{MjcfError, load_model};

const CASES: &[(&str, [u64; 3], [u64; 4])] = &[
    // Two equal moments and an off-diagonal below 1e-12: MuJoCo stops at once,
    // the frame untouched (a turn would be 45° here).
    (
        "1 1 0.5 5e-13 0 0",
        [0x3ff0000000000000, 0x3ff0000000000000, 0x3fe0000000000000],
        [0x3ff0000000000000, 0, 0, 0],
    ),
    (
        "1 2 3 0 0 0",
        [0x4008000000000000, 0x4000000000000000, 0x3ff0000000000000],
        [
            0x0000000000000000,
            0x3fe6a09e667f3bd9,
            0x0000000000000000,
            0x3fe6a09e667f3bd9,
        ],
    ),
    (
        "1 3 2 0 0 0",
        [0x4008000000000000, 0x4000000000000000, 0x3ff0000000000000],
        [
            0x3fe0000000000006,
            0x3fe0000000000006,
            0x3fe0000000000006,
            0x3fe0000000000006,
        ],
    ),
    (
        "2 1 3 0 0 0",
        [0x4008000000000000, 0x4000000000000000, 0x3ff0000000000000],
        [
            0x3fe0000000000006,
            0x3fe0000000000006,
            0xbfe0000000000006,
            0x3fe0000000000006,
        ],
    ),
    (
        "2 3 1 0 0 0",
        [0x4008000000000000, 0x4000000000000000, 0x3ff0000000000000],
        [
            0x3fe6a09e667f3bd1,
            0x0000000000000000,
            0x0000000000000000,
            0x3fe6a09e667f3bd1,
        ],
    ),
    (
        "3 1 2 0 0 0",
        [0x4008000000000000, 0x4000000000000000, 0x3ff0000000000000],
        [
            0x3fe6a09e667f3bd1,
            0x3fe6a09e667f3bd1,
            0x0000000000000000,
            0x0000000000000000,
        ],
    ),
    (
        "3 2 1 0 0 0",
        [0x4008000000000000, 0x4000000000000000, 0x3ff0000000000000],
        [
            0x3ff0000000000000,
            0x0000000000000000,
            0x0000000000000000,
            0x0000000000000000,
        ],
    ),
    (
        "2 2 2 0.5 0 0",
        [0x4004000000000001, 0x4000000000000000, 0x3ff8000000000000],
        [
            0x3fe4e7ae9144f104,
            0x3fe4e7ae9144f104,
            0x3fd1517a7bdb389b,
            0x3fd1517a7bdb389b,
        ],
    ),
    (
        "1 1 2 0 0 0",
        [0x4000000000000000, 0x3ff0000000000000, 0x3ff0000000000000],
        [
            0x3fe0000000000006,
            0x3fe0000000000006,
            0xbfe0000000000006,
            0x3fe0000000000006,
        ],
    ),
    (
        "2 1 1 0 0 0",
        [0x4000000000000000, 0x3ff0000000000000, 0x3ff0000000000000],
        [
            0x3ff0000000000000,
            0x0000000000000000,
            0x0000000000000000,
            0x0000000000000000,
        ],
    ),
    (
        "1 2 1 0 0 0",
        [0x4000000000000000, 0x3ff0000000000000, 0x3ff0000000000000],
        [
            0x3fe6a09e667f3bd1,
            0x0000000000000000,
            0x0000000000000000,
            0x3fe6a09e667f3bd1,
        ],
    ),
    (
        "0.03176329093621826 0.018974565395872785 0.033850717151490226 -0.0018028339944212775 0.0005346670868071647 0.0008814936607019899",
        [0x3fa167a19fd997f8, 0x3fa058f0cd8bbc33, 0x3f931d3e381e1221],
        [
            0x3fe30efa8ea00487,
            0x3fdff58b22cb76e3,
            0xbfddfc3081e5810b,
            0x3fdae195374a48bf,
        ],
    ),
    (
        "0.13622757087298526 0.1563147609264945 0.19521797534573873 0.048007532809750404 -0.004837793662646921 0.005948349547178536",
        [0x3fc93ba927173256, 0x3fc8d42821d8965e, 0x3fb8be39759a3159],
        [
            0x3fde219e581bd822,
            0x3fe5bfcd8a18c6b0,
            0x3f82be4933ba0c60,
            0x3fe1ff5e1bb84f35,
        ],
    ),
    (
        "0.14900235674099513 0.31272304709115534 0.19498937293038973 -0.045781050862942936 -0.1322392876529249 -0.03779103842890461",
        [0x3fd4cce9ff8b579b, 0x3fd398d8a1399a88, 0x3f9a1dac97abcc20],
        [
            0x3fe32050fd2db6f7,
            0x3fce54ff1ee5aaed,
            0x3fd6f83aa29e0045,
            0x3fe5a6c37bf8c228,
        ],
    ),
    (
        "0.3043292595750755 0.11891752521302712 0.3207797063517776 0.057193759402095055 0.018323292154692646 -0.030491366549495347",
        [0x3fd54e65e9ef6902, 0x3fd413cd5807578d, 0x3fb8efb81ef2aaac],
        [
            0x3fe11c5fd7055ba8,
            0x3fe7058f4e9921f3,
            0xbfcef65ce20df6dc,
            0x3fd7c63158b067ae,
        ],
    ),
    (
        "0.31149996132331476 0.27510784486465883 0.2609939773564306 -0.06194647342999963 -0.0739555043629731 0.0513871596309463",
        [0x3fda587ee432c7b7, 0x3fcd520c845fc647, 0x3fca7b2ccd680a0e],
        [
            0x3fed6c81a313fc06,
            0x3f9cc9ffbb7f5410,
            0x3fd14bc3b162fecf,
            0xbfd22e6a0fe3782b,
        ],
    ),
    (
        "0.13069266358590936 0.1152490528849482 0.12191989940797374 -0.0003594097095357509 -0.0009281005526476855 0.019018034516076924",
        [0x3fc1aa71400ca25f, 0x3fc0b6ac6aaea970, 0x3fb969f27430a6ad],
        [
            0x3fe31ad69ee522a7,
            0x3fd212943dc661f4,
            0xbfd418f91adea244,
            0x3fe5d3728a34220b,
        ],
    ),
    (
        "0.061990075675305865 0.08121073698241403 0.08413256061069117 -0.03643855539188229 -0.014234190255415113 -0.006670709830644689",
        [0x3fbc1bbdc6ac360d, 0x3fb683cf0c501475, 0x3f9e4be121f7797e],
        [
            0x3fd32f4cd318eb5a,
            0x3fd3e401bf82f69c,
            0x3fe08e3860869aa8,
            0x3fe7a49a5cb5727a,
        ],
    ),
    (
        "0.3196878321540493 0.3681516564188041 0.31410439660081035 0.020830822618603546 0.05568260742766085 0.016728318870947813",
        [0x3fd96aa673de9157, 0x3fd5ff6f4e2f1b15, 0x3fd0b5c37cc233ce],
        [
            0x3febb375467dbec0,
            0xbfc299b290e5f70c,
            0xbfd6081f4f3cf48d,
            0x3fd55305ae29c4c5,
        ],
    ),
    (
        "0.24031940289280776 0.387959262135299 0.2591324394948128 0.06141678569161569 -0.01794371386324851 -0.001721841143231136",
        [0x3fda46b1642b85ce, 0x3fd0ecbc91868360, 0x3fcb2fd3f8a02cce],
        [
            0x3fe5ab6971395af8,
            0x3fdd8e79b5982db9,
            0x3fd6487a3085cef9,
            0x3fdd1d08f211cb2f,
        ],
    ),
    (
        "0.28375879963158734 0.3150885704500084 0.4963860675194377 0.08083080251210974 -0.04531727881393845 -0.015595842869593443",
        [0x3fe05d5fd6b786f0, 0x3fd79e94ff993f4c, 0x3fcb7df29fed1afa],
        [
            0x3fc1274bed73e90a,
            0x3fe3147733c87ecb,
            0xbfd418cbff55de82,
            0x3fe7402d3eada790,
        ],
    ),
    (
        "0.10264059670298063 0.09800048752533118 0.07282427070422609 0.008164518398539078 0.039001579264630756 -0.02424021750821632",
        [0x3fc0d06ffbe321bd, 0x3fbb21e8afc3f5c3, 0x3fa27e1558f7fc2a],
        [
            0x3fed9acd2fc65955,
            0xbfc20eb629fd7541,
            0xbfd3341ad6caf31d,
            0xbfc7a916267dad5f,
        ],
    ),
    (
        "0.39048418663496537 0.29165279163541125 0.4097488661903564 -0.03401030533967856 -0.13404686276952799 0.02865358434347015",
        [0x3fe15a6c24f36935, 0x3fd2427999795a50, 0x3fd0ea2346daf93e],
        [
            0x3f9ba16bb3ba6200,
            0x3fd9cdd3d8e29e07,
            0x3fc3d2c817d0e582,
            0x3fecd93325c73e33,
        ],
    ),
    (
        "0.09440250259366983 0.11161481200427881 0.11527419953250195 -0.0010019561237941137 -0.03206181370992348 0.00311441020309017",
        [0x3fc1c7ce40c6befe, 0x3fbc7d41a2b9ccae, 0x3fb2334afd64e546],
        [
            0x3facce631e7a5470,
            0x3fdd0578a7cf9b06,
            0x3f8db1ba2bbabb10,
            0x3fec75d00e4f31f6,
        ],
    ),
    (
        "0.1298274282612351 0.1972931768048332 0.17845657750710747 0.03187051850095163 -0.0583710600090909 -0.011718937078040352",
        [0x3fce65696d330d0c, 0x3fc72c1f08f19318, 0x3fb64a70ac782f6d],
        [
            0xbfcd81d5f8399236,
            0x3fdbec7312caa24a,
            0xbfcdb80ce0b72438,
            0x3fead29333ca05b4,
        ],
    ),
    (
        "0.055197004125709416 0.08277148012148551 0.046081414208140464 -0.0013532097890300344 0.0344570022856688 0.0050837299954423236",
        [0x3fb63b6cae12e5cd, 0x3fb4e7527e417273, 0x3f8fd92dd2f24f2b],
        [
            0x3fecaf6f31ad3f9a,
            0xbfb0976834f08c7a,
            0xbfd629fd0c0787d7,
            0x3fd135a400bfb8a9,
        ],
    ),
    (
        "0.30135608635429934 0.41975005268768967 0.31796903430542417 -0.050601989822799474 0.0222964215861068 -0.05917589420726644",
        [0x3fddcb3baa0daf4f, 0x3fd29dea83bb9522, 0x3fd2170efa1df184],
        [
            0x3fe02dd873e78864,
            0x3fd2a2b80612705c,
            0x3fe52842819b4c1c,
            0x3fde2f65e440f374,
        ],
    ),
    (
        "0.3387349750005642 0.19041725329867204 0.23926321087569585 0.023710255877742947 0.059740195857678965 0.012834417307068908",
        [0x3fd7ba60e34b7428, 0x3fcb0dda3d6d929c, 0x3fc7d8d3e27b3d69],
        [
            0x3fe70aee5da3e0d7,
            0x3fe4f553290671a5,
            0xbfbbe50fc530d16c,
            0x3fc9cea114e60210,
        ],
    ),
    (
        "0.36885776615144167 0.3500542896593826 0.20315040602924953 0.03788136709760086 -0.04365557002197898 -0.016816963126940806",
        [0x3fda2259a05a8b74, 0x3fd496f3aa60b71e, 0x3fc89389f5c5f148],
        [
            0x3fee4401195c11bb,
            0x3f8893b9c3896484,
            0x3fbe1699790959f3,
            0x3fd35c0baf553e7a,
        ],
    ),
    (
        "0.38503415037115984 0.28321633816961217 0.12291098164775657 -0.020724597304529937 0.004140582133052461 0.08745606418370648",
        [0x3fd8f434c7c4861b, 0x3fd44eddb1fd1e9f, 0x3fb57d44f94a116a],
        [
            0x3fef0ac34c3b3a5e,
            0x3fcaa01ad5a81b24,
            0x3f7891d9ea0f995c,
            0xbfc0061aafb9adb5,
        ],
    ),
    (
        "0.427442865091485 0.3274135621810053 0.3570129811283753 0.038068802196385476 -0.062380895857957364 -0.02618855130532233",
        [0x3fde971962688a82, 0x3fd499112514d994, 0x3fd3f8b3c722e706],
        [
            0x3fe048cf4605c9e2,
            0x3fe9d83f4291ea7e,
            0x3fd1045831904d87,
            0xbfc13153a5c6c8d6,
        ],
    ),
    (
        "0.2638046518433766 0.1489501100818132 0.16456420302791264 -0.01595941027656587 -0.00845055189817248 -0.008689441279453779",
        [0x3fd10e3a922c9a67, 0x3fc58ef36b0863bb, 0x3fc23a2ded892cca],
        [
            0x3fe144f32dcd8681,
            0x3fead616821686db,
            0xbfa1fe697a8c5a9a,
            0xbfb0a05a4f0e99f4,
        ],
    ),
    (
        "0.24767903122592116 0.3470546945754537 0.21372278383882584 -0.019594356832678073 -0.00014702100724226297 -0.020498856274768724",
        [0x3fd6a28e3dfb07a5, 0x3fcf48e808edf26e, 0x3fcaed7c39ae4b1f],
        [
            0x3fe47c3a2d98b1b7,
            0xbf924596169e0a5a,
            0x3fb706c9177d099e,
            0x3fe868690e48a0eb,
        ],
    ),
    (
        "0.49266633388928777 0.4010969053814457 0.41533647414401764 -0.04598120870202522 -0.055704973191221106 0.020707470874442496",
        [0x3fe14821e23ec128, 0x3fd8d13bd501689c, 0x3fd866ca9095850f],
        [
            0x3fe090ffbd9a9588,
            0x3fe9ccb2fef6ba20,
            0xbf9d0eb6639e5a08,
            0xbfd23ba2617ed094,
        ],
    ),
    (
        "0.1096205357528643 0.13827726894972775 0.21121906519853365 0.0015349906127109846 0.01357556799993263 -0.029974631450092784",
        [0x3fcc948f3b11bcd8, 0x3fc08b2d6fdd622c, 0x3fbb49358f662aa0],
        [
            0xbf96a8bb14ccff00,
            0x3fe7cda67362d854,
            0xbfc9df6ab2d476d0,
            0x3fe45f836351faaf,
        ],
    ),
    (
        "0.2051110265066498 0.24677887031930593 0.33716061390620483 0.07254394612875559 -0.008858713839950915 -0.060947535608284205",
        [0x3fd83d25dff276c2, 0x3fd0f44a6dba67ac, 0x3fc29cbad1e49039],
        [
            0x3fa1c05fc6c9d720,
            0x3fe374de21350bcd,
            0xbfdda3469463e616,
            0x3fe49b2c9d02a97a,
        ],
    ),
    (
        "0.349627985066287 0.14279788912903058 0.34682990339159137 -0.06679501071209397 -0.010371075956779274 0.0724126060341882",
        [0x3fd9646b4f5b3536, 0x3fd5a0b3f2d0ae82, 0x3fbac4fa6d07700f],
        [
            0x3fd8ac73f3cb59b0,
            0x3fc333ebef8e2845,
            0xbfe2c0465ec86588,
            0x3fe64c03e59004d2,
        ],
    ),
    (
        "0.10752439630521486 0.08554100917301244 0.10438532581022722 -0.04600465419806401 0.009631006490961411 0.02982602521202012",
        [0x3fc2db814d453939, 0x3fbd213f62e97a35, 0x3fa29af25b346959],
        [
            0x3fe7c0e7d1fc3836,
            0x3fe1535bbbbe6144,
            0xbfb3f4a107508d82,
            0xbfd8c4bf053506aa,
        ],
    ),
    (
        "0.20076314893350064 0.16293185513487707 0.21447999352827296 -0.0696029468184994 0.0453468326026598 0.09337356211245369",
        [0x3fd252ded64a02fd, 0x3fcf53d80e0fc658, 0x3fa82036b9869026],
        [
            0x3fe4dd484b81d7a2,
            0x3fbf3540ffd81c22,
            0xbfde40aeaf067b4a,
            0x3fe2909a1612070c,
        ],
    ),
    (
        "0.2570289732995913 0.2932274936121609 0.40864465210411915 -0.02855941962673029 -0.049509620729997955 0.09182986767686224",
        [0x3fde5cd6e2df73cf, 0x3fcf23ed2e7bfe00, 0x3fcedfaaa4e7c9bf],
        [
            0xbfa9b8cd3bdbe730,
            0x3fe37960135faeab,
            0x3fdc2191a8d958a6,
            0x3fe514488e0812c8,
        ],
    ),
    (
        "0.19133310414023405 0.16199276666298718 0.2749961798374883 -0.0455232767400989 0.021533190307226367 0.03683078603336644",
        [0x3fd2595c6e24b082, 0x3fccb38712cc334f, 0x3fbe0d36e4b05ca0],
        [
            0x3fe6327bdaa2c9c4,
            0x3fd0ba612f1dda50,
            0xbfe1925787746f2e,
            0x3fd8b41223a3c6a7,
        ],
    ),
    (
        "0.2505131085479391 0.23246266791812736 0.21411782120104425 0.06118731628570207 -0.05631688283099395 -0.0445951990697416",
        [0x3fd5f7bd22347b70, 0x3fc721b8b9613be0, 0x3fc62929f05365f8],
        [
            0x3fed1484d5414bdf,
            0x3fb6b9c88f24047d,
            0x3fd2df72602f92d7,
            0x3fd206830a99032f,
        ],
    ),
    (
        "0.26779981499813815 0.13802492413809736 0.25351881160192863 0.035407798725186304 0.06589122757045984 -0.05480947607014517",
        [0x3fd4f7684f708899, 0x3fce3b7bd4b942ca, 0x3fb87624440b1206],
        [
            0x3fdba93b6a88518a,
            0x3fea450515ab718b,
            0xbfca1992d8d7fdb6,
            0x3fd4013f88d91601,
        ],
    ),
    (
        "0.0934358712642505 0.21551645931439822 0.283060971201326 -0.033686409851403545 0.01309225130464545 -0.0012396212629820784",
        [0x3fd22fb14ed12ef8, 0x3fcca792eb9464e0, 0x3fb58043f7839f62],
        [
            0xbfbc4c2875d744d4,
            0x3fe73a885640577e,
            0x3fb06702beeacf14,
            0x3fe5a0704d3903b8,
        ],
    ),
    (
        "0.1811251387919401 0.11490893478776802 0.12734785960889458 0.0137157361103688 0.039406307061105865 0.006030720562826252",
        [0x3fca2b00d1382e9e, 0x3fbcd34f2c2faca0, 0x3fbb397156231c74],
        [
            0x3feeca37105f7f2b,
            0xbfba77ad778e7459,
            0xbfcf5a958a0ec398,
            0x3fae55db3aef3c53,
        ],
    ),
    (
        "0.2970055809772456 0.3195794465296807 0.2422096267701116 0.06654760202763847 0.08939911054629278 0.04062037142459752",
        [0x3fdb0327359a79a2, 0x3fd0b50924f03d76, 0x3fc67c9afff74e6e],
        [
            0x3fecc348e712c44e,
            0xbfc0d57d1f2bfbc5,
            0xbfd31c65a5b0fcc5,
            0x3fd2ba48016b70c1,
        ],
    ),
    (
        "0.22084632791937236 0.1823463387346723 0.18737260772697545 0.023391028544841787 -0.021807235538484442 0.009832478098987536",
        [0x3fce5e8a6d116f0c, 0x3fc8f2769f10cb8e, 0x3fc446a389ad193f],
        [
            0x3fec456fa781e4f0,
            0x3fd9ef58d555fbab,
            0x3fcc4a26fb84d643,
            0x3fb47e923d2107e6,
        ],
    ),
    (
        "0.28916905687616723 0.2980975498057138 0.07289150266207883 0.019021748078234105 -0.0789271972453825 0.07677327918537051",
        [0x3fd4be5bc05d096c, 0x3fd3f1d5673c7b54, 0x3f98fd6a4db0eafc],
        [
            0x3fe012cf241ba88a,
            0x3fc9bb2e4bcfd90f,
            0xbfa6e56e78c7c0c4,
            0x3feadfc20e9e0290,
        ],
    ),
    (
        "0.04420884811074235 0.04801608978248366 0.0631837214961685 2.1269724343733864e-05 -0.0002387016037923278 -0.0074309104811685865",
        [0x3fb0f3c8cc06e168, 0x3fa708c23f072824, 0x3fa6a165741a8537],
        [
            0xbfc54adc9a685156,
            0x3fe5def4b36eb39c,
            0xbfbafb06f36a4f74,
            0x3fe67e4f8755b2bc,
        ],
    ),
    (
        "0.2206840471210225 0.2650572444325054 0.3551061939557387 0.033119061766112026 -0.1090336730661183 0.04229370542825171",
        [0x3fdad06862df346e, 0x3fd20b1a7d5d5e84, 0x3fc1e9de311ead47],
        [
            0x3fc454f377a5216e,
            0x3fdfb5b2ab4d0224,
            0xbfc0439ab6ee5bf3,
            0x3feb05f104c51602,
        ],
    ),
    (
        "0.3065805982085362 0.18286578059264208 0.3173382405631612 0.08659073512282682 0.06909220719479114 -0.037950610576854366",
        [0x3fd8cf5c644ae00b, 0x3fd363c5e1ff3390, 0x3fbdbce6b9ed9e78],
        [
            0x3fde16de14417eda,
            0x3fe9e2a79ab10bdf,
            0xbfb381cc7bdc3220,
            0x3fd60f5717e0ea91,
        ],
    ),
    // Tied off-diagonals: the pivot is the first of the largest.
    (
        "2 2 2 0.5 0.5 0.5",
        [0x4007ffffffffffff, 0x3ff8000000000000, 0x3ff8000000000000],
        [
            0x3fea415117dce1ea,
            0x3fd5c01f0226dd78,
            0xbfc6848b0d51d025,
            0x3fdb2e6b4da25558,
        ],
    ),
    (
        "1 1 1 0.3 0.3 0",
        [0x3ff6c9c91ebfc524, 0x3fefffffffffffff, 0x3fe26c6dc28075b9],
        [
            0x3fd6a09e60db383c,
            0x3feb504f346aace0,
            0x3fc2bec337ac7f40,
            0x3fd6a09e6587c81e,
        ],
    ),
    (
        "1 1 1 0.3 0 0.3",
        [0x3ff6c9c91ebfc524, 0x3fefffffffffffff, 0x3fe26c6dc28075b9],
        [
            0x3fe6a09e6825f110,
            0x3fdffffffb53f3f6,
            0xbe2a70c6e8000000,
            0x3fe0000000003974,
        ],
    ),
];

fn inertial_doc(full: &str) -> String {
    format!(
        r#"<mujoco>
            <worldbody>
                <body name="b">
                    <freejoint/>
                    <inertial pos="0 0 0" mass="1" fullinertia="{full}"/>
                </body>
            </worldbody>
        </mujoco>"#
    )
}

#[test]
fn a_full_inertias_principal_moments_and_frame_are_mujocos_bit_for_bit() {
    let mut wrong = Vec::new();
    for (full, inertia, iquat) in CASES {
        let model = load_model(&inertial_doc(full)).unwrap();
        let got_inertia = model.body_inertia[1].map(f64::to_bits);
        let q = model.body_iquat[1];
        let got_iquat = [q.w, q.i, q.j, q.k].map(f64::to_bits);
        if got_inertia.as_slice() != inertia || got_iquat != *iquat {
            wrong.push(*full);
        }
    }
    assert!(
        wrong.is_empty(),
        "{} of {} differ: {wrong:?}",
        wrong.len(),
        CASES.len()
    );
}

#[test]
fn a_full_inertia_mujoco_refuses_is_refused_with_its_text() {
    for full in ["1 1 1 2 0 0", "1 1 1e-15 0 0 0"] {
        match load_model(&inertial_doc(full)).unwrap_err() {
            MjcfError::InvalidValue {
                element,
                attribute,
                reason,
                ..
            } => {
                assert_eq!(
                    (element.as_str(), attribute.as_str(), reason.as_str()),
                    (
                        "inertial",
                        "fullinertia",
                        "inertia must have positive eigenvalues"
                    ),
                    "{full}"
                );
            }
            other => panic!("{full}: {other}"),
        }
    }
}

/// Asserts that `xml` is refused for its body 'b''s fullinertia.
fn assert_full_inertia_refused(xml: &str) {
    let err = load_model(xml).unwrap_err();
    assert!(
        matches!(&err, MjcfError::InvalidValue { element, attribute, .. }
            if element == "inertial" && attribute == "fullinertia"),
        "{err}"
    );
    assert!(err.to_string().ends_with("(body 'b')"), "{err}");
}

#[test]
fn a_non_finite_full_inertia_is_refused_not_a_hang() {
    // MuJoCo 3.5.0 refuses the first two on a moving body (its mass and
    // inertia minimum) and loads them on a jointless one, with a NaN inertia
    // and frame, or for a NaN first value, which it reads as no fullinertia,
    // a zero inertia. It loads the third with an infinite moment.
    for full in [
        "1 1 1 nan 0 0",
        "nan 1 1 0 0 0",
        "1.5e308 1.5e308 1.5e308 1e308 0 0",
    ] {
        assert_full_inertia_refused(&inertial_doc(full));
    }
    for full in ["1 1 1 nan 0 0", "1 1 1 inf 0 0", "nan 1 1 0 0 0"] {
        assert_full_inertia_refused(&format!(
            r#"<mujoco>
                <worldbody>
                    <body name="b"><inertial pos="0 0 0" mass="1" fullinertia="{full}"/></body>
                </worldbody>
            </mujoco>"#
        ));
    }
}

#[test]
fn an_unnamed_bodys_inertia_error_names_it_by_its_index() {
    // The world is body #0.
    let err = load_model(
        r#"<mujoco>
            <worldbody>
                <body><inertial pos="0 0 0" mass="1" fullinertia="1 1 1 2 0 0"/></body>
            </worldbody>
        </mujoco>"#,
    )
    .unwrap_err();
    assert!(err.to_string().ends_with("(body #1)"), "{err}");
}

#[test]
fn a_full_inertia_replaced_by_the_geoms_is_still_refused_where_mujoco_refuses_it() {
    // MuJoCo makes every fullinertia principal before inertiafromgeom="true"
    // replaces it: its refusal stands, and a NaN one, which the decomposition
    // passes, is replaced (both measured on MuJoCo 3.5.0).
    let doc = |inertial: &str| {
        format!(
            r#"<mujoco>
                <compiler inertiafromgeom="true"/>
                <worldbody>
                    <body name="b"><freejoint/>{inertial}<geom size="0.1"/></body>
                </worldbody>
            </mujoco>"#
        )
    };
    assert_full_inertia_refused(&doc(
        r#"<inertial pos="0 0 0" mass="1" fullinertia="1 1 1 2 0 0"/>"#,
    ));
    let replaced = load_model(&doc(
        r#"<inertial pos="0 0 0" mass="1" fullinertia="1 1 1 nan 0 0"/>"#,
    ))
    .unwrap();
    let geoms_only = load_model(&doc("")).unwrap();
    assert_eq!(
        replaced.body_inertia[1].map(f64::to_bits),
        geoms_only.body_inertia[1].map(f64::to_bits)
    );
}

#[test]
fn a_body_whose_geoms_mass_or_inertia_overflows_is_refused_not_a_hang() {
    // MuJoCo 3.5.0 loads all three: size 1e120 with an infinite mass and
    // inertia, size 6e99 with a finite mass (9.05e302) and an infinite
    // inertia, and two unit spheres far apart, whose summed tensor is finite,
    // with two infinite principal moments.
    for geoms in [
        r#"<geom size="1e120"/>"#,
        r#"<geom size="6e99"/>"#,
        r#"<geom size="1" pos="8.92e151 8.92e151 8.92e151"/>
           <geom size="1" pos="-8.92e151 -8.92e151 -8.92e151"/>"#,
    ] {
        let err = load_model(&format!(
            r#"<mujoco>
                <worldbody>
                    <body name="huge"><freejoint/>{geoms}</body>
                </worldbody>
            </mujoco>"#
        ))
        .unwrap_err();
        assert!(
            matches!(&err, MjcfError::InvalidModel { reason, .. } if reason.contains("not finite")),
            "{geoms}: {err}"
        );
        assert!(err.to_string().ends_with("(body 'huge')"), "{geoms}: {err}");
    }
}
