# ==========================================
# Makefile pour Sous-Répertoires
# ==========================================

CC = gcc
CFLAGS = -fopenmp -Wall -Wextra -g -O2 -Iinclude
LIBS = -lm

# Dossiers
SRCDIR = src
APPDIR = apps
INCDIR = include
OBJDIR = obj

UTILS_SRCS = $(shell find $(SRCDIR) -name "*.c")

UTILS_OBJS = $(patsubst %.c, $(OBJDIR)/%.o, $(notdir $(UTILS_SRCS)))

VPATH = $(shell find $(SRCDIR) -type d)

APP_SRCS = $(wildcard $(APPDIR)/*.c)
TARGETS = $(patsubst $(APPDIR)/%.c, %, $(APP_SRCS))

all: $(OBJDIR) $(TARGETS)

$(OBJDIR):
	mkdir -p $(OBJDIR)

$(OBJDIR)/%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%: $(APPDIR)/%.c $(UTILS_OBJS)
	$(CC) $(CFLAGS) $< $(UTILS_OBJS) -o $@ $(LIBS)
	@echo "--- Exécutable $@ créé ---"

clean:
	rm -rf $(OBJDIR) $(TARGETS)

.PHONY: all clean
