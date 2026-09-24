plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
}

android {
    namespace = "com.mojotech.mojodrive"
    compileSdk = 35

    defaultConfig {
        applicationId = "com.mojotech.mojodrive"
        minSdk = 26
        targetSdk = 35
        versionCode = 9
        versionName = "0.9"
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    kotlinOptions {
        jvmTarget = "17"
    }
}
