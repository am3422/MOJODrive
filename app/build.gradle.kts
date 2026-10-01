plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
}

android {
    namespace = "com.mojotech.mojodrive"
    compileSdk = 35

    defaultConfig {
        applicationId = "com.mojotech.mojodrive.farsv2"
        minSdk = 26
        targetSdk = 35
        versionCode = 16
        versionName = "0.15-fars-direction-v2"
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    kotlinOptions {
        jvmTarget = "17"
    }
}


dependencies {
    implementation("com.google.android.gms:play-services-location:21.3.0")
}
