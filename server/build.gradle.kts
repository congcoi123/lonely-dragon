import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

plugins {
    id("idea")
    id("java")
    kotlin("jvm") version "1.4.30"
    id("io.gitlab.arturbosch.detekt") version "1.18.1"
}

group = "com.congcoi123.lonely.dragon"
version = "0.0.1"
java.sourceCompatibility = JavaVersion.VERSION_11

repositories {
    mavenLocal()
    mavenCentral()
}

dependencies {
    // tenio
    implementation("io.github.congcoi123:tenio-core:0.0.4")
    implementation("io.github.congcoi123:tenio-engine:0.0.2")

    // detekt
    implementation("org.jetbrains.kotlinx:kotlinx-html-jvm:0.7.3")
}

tasks.test {
    useJUnitPlatform()
}

tasks.withType<KotlinCompile> {
    kotlinOptions.jvmTarget = "11"
}

tasks.jar {
    manifest {
        attributes["Main-Class"] = "com.congcoi123.lonely.dragon.LonelyDragonKt"
    }
    configurations["compileClasspath"].forEach { file: File ->
        from(zipTree(file.absoluteFile))
    }
    duplicatesStrategy = DuplicatesStrategy.INCLUDE
}

detekt {
    reports {
        xml.enabled = false
        html.enabled = false
    }
    config = this.config.from("detekt-config.yml")
    parallel = true
    buildUponDefaultConfig = true
}
