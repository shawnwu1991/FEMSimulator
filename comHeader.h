#pragma once
#include <QtWidgets>

//inline QString ResourcePath(const QString &str) { return str; }
inline QString ResourcePath(const QString &str) { return QStringLiteral(":/Shaders/") + str; }
