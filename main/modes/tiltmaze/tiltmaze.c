/**
 * @file tiltmaze.c
 * @author dylwhich (dylan@whichard.com)
 * @brief A virtual ball-in-maze game
 * @date 2023-08-02
 *
 */

//==============================================================================
// Includes
//==============================================================================

#include "esp_random.h"
#include "esp_log.h"
#include "tiltmaze.h"
#include "trigonometry.h"
#include "linked_list.h"

//==============================================================================
// Defines
//==============================================================================

#define BALL_RADIUS 5
#define GRAVITY     (980) // 9.80m/s or... 980cm/s?

/// Control Scheme defines
#define BUTTON_INCREMENT 50

//==============================================================================
// Enums
//==============================================================================

/**
 * @brief Enum of screens that may be shown in MAZE mode
 */
typedef enum
{
    MAZE_MENU,
    MAZE_GAME,
} mazeScreen_t;

/**
 * @brief Enum of control schemes for a maze game
 */
typedef enum
{
    MAZE_BUTTON,
    MAZE_TOUCH,
    MAZE_TILT,
} mazeControl_t;

/**
 * @brief Enum of level generation difficulties
 */
typedef enum
{
    MAZE_EASY,
    MAZE_MEDIUM,
    MAZE_HARD,
} mazeDifficulty_t;

/**
 * @brief Enum of different object types that can be in a maze
 *
 */
typedef enum
{
    MAZE_PEG,
    MAZE_HOLE,
    MAZE_WALL,
    MAZE_GOAL,
} mazeEntityType_t;

/**
 * @brief Structure defining an obstacle in the maze
 */
typedef struct
{
    paletteColor_t color;
    mazeEntityType_t type;
    union
    {
        circle_t peg;
        circle_t hole;
        rectangle_t wall;
    };
} mazeEntity_t;

typedef struct
{
    int32_t width;  ///< The width of the maze
    int32_t height; ///< The height of the maze

    int32_t startX; ///< The starting X position of the ball
    int32_t startY; ///< The starting Y position of the ball

    int16_t ballMass;     ///< The mass of the ball, in... grams?
    int16_t ballFriction; ///< The ball coefficient of friction

    list_t obstacles; ///< The list of obstacles in the maze

} mazePuzzle_t;

//==============================================================================
// Structs
//==============================================================================

typedef struct
{
    menu_t* menu;                               ///< The menu structure
    menuLogbookRenderer_t* menuLogbookRenderer; ///< Renderer for the menu
    font_t ibm;                                 ///< The font used in the menu and game
    p2pInfo p2p;                                ///< Peer to peer connectivity info, currently unused
    mazeScreen_t screen;                        ///< The screen being displayed

    mazeControl_t control;       ///< The selected control scheme
    mazeDifficulty_t difficulty; ///< The selected CPU difficulty

    circle_t ball; ///< The ball
    vec_t ballVel; ///< The ball's velocity

    mazePuzzle_t puzzle; ///< The currently-loaded puzzle

    int32_t restartTimerUs; ///< A timer that counts down before the game begins

    uint16_t btnState; ///< The button state used for paddle control
    int16_t tiltH;     ///< The angle of the left-to-right tilt, in 100ths-of-degrees off-level
    int16_t tiltV;     ///< The angle of the top-to-bottom tilt, in 100ths-of-degrees off-level
    bool isPaused;     ///< true if the game is paused, false if it is running

    wsg_t ballWsg; ///< Ball sprite

    song_t bgm;  ///< Background music
    song_t hit1; ///< Sound effect for when the ball hits a wall
} maze_t;

//==============================================================================
// Function Prototypes
//==============================================================================

static void mazeMainLoop(int64_t elapsedUs);
static void mazeEnterMode(void);
static void mazeExitMode(void);
static void mazeEspNowRecvCb(const esp_now_recv_info_t* esp_now_info, const uint8_t* data, uint8_t len, int8_t rssi);
static void mazeEspNowSendCb(const uint8_t* mac_addr, esp_now_send_status_t status);
static void mazeMenuCb(const char* label, bool selected, uint32_t settingVal);
static void mazeGameLoop(int64_t elapsedUs);

static void mazeResetGame(void);
static void mazeHandlePlayerInput(void);
static void mazeUpdatePhysics(int64_t elapsedUs);

static void mazeBackgroundDrawCallback(int16_t x, int16_t y, int16_t w, int16_t h, int16_t up, int16_t upNum);
static void mazeDrawField(void);

//==============================================================================
// Strings
//==============================================================================

/* Design Pattern!
 * These strings are all declared 'const' because they do not change, so that they are placed in ROM, not RAM.
 * Lengths are not explicitly given so the compiler can figure it out.
 */

static const char mazeName[] = "TiltMaze";

static const char mazeCtrlButton[] = "Button Control";
static const char mazeCtrlTouch[]  = "Touch Control";
static const char mazeCtrlTilt[]   = "Tilt Control";

static const char mazeDiffEasy[]   = "Easy";
static const char mazeDiffMedium[] = "Medium";
static const char mazeDiffHard[]   = "Impossible";

static const char mazePaused[] = "Paused";

//==============================================================================
// Variables
//==============================================================================

/// The Swadge mode for TiltMaze
swadgeMode_t mazeMode = {
    .modeName                 = mazeName,
    .wifiMode                 = ESP_NOW,
    .overrideUsb              = false,
    .usesAccelerometer        = true,
    .usesThermometer          = false,
    .fnEnterMode              = mazeEnterMode,
    .fnExitMode               = mazeExitMode,
    .fnMainLoop               = mazeMainLoop,
    .fnAudioCallback          = NULL,
    .fnBackgroundDrawCallback = mazeBackgroundDrawCallback,
    .fnEspNowRecvCb           = mazeEspNowRecvCb,
    .fnEspNowSendCb           = mazeEspNowSendCb,
    .fnAdvancedUSB            = NULL,
};

/// All state information for the TiltMaze mode.
maze_t* maze = NULL;

//==============================================================================
// Functions
//==============================================================================

/**
 * @brief Enter TiltMaze mode, allocate required memory, and initialize required variables
 *
 */
static void mazeEnterMode(void)
{
    // Allocate and clear all memory for this mode. All the variables are contained in a single struct for convenience.
    // calloc() is used instead of malloc() because calloc() also initializes the allocated memory to zeros.
    maze = calloc(1, sizeof(maze_t));

    // Below, various assets are loaded from the SPIFFS file system to RAM. How did they get there?
    // The source assets are found in the /assets/maze/ directory. Each asset is processed and packed into the SPIFFS
    // file system at compile time The following transformations are made:
    // * pball.png   -> pball.wsg
    // * ppaddle.png -> ppaddle.wsg
    // * block1.mid  -> block1.sng
    // * block2.mid  -> block2.sng
    // * gmcc.mid    -> gmcc.sng
    //
    // In addition, a common font is found in /assets/fonts/ and is transformed like so:
    // * ibm_vga8.font.png -> ibm_vga8.font
    //
    // If you'd like to learn more about how assets are processed and packed, see
    // /tools/spiffs_file_preprocessor/README.md
    //
    // If you'd like to learn more about how assets are loaded, see
    // /components/hdw-spiffs/include/hdw-spiffs.h

    // Load a font
    loadFont("ibm_vga8.font", &maze->ibm, false);

    // Load graphics
    loadWsg("pball.wsg", &maze->ballWsg, false);

    // Load SFX
    loadSong("block1.sng", &maze->hit1, false);
    loadSong("gmcc.sng", &maze->bgm, false);
    maze->bgm.shouldLoop = true;

    // Initialize the menu
    maze->menu                = initMenu(mazeName, mazeMenuCb);
    maze->menuLogbookRenderer = initMenuLogbookRenderer(&maze->ibm);

    // These are the possible control schemes
    const char* controlSchemes[] = {
        mazeCtrlButton,
        mazeCtrlTouch,
        mazeCtrlTilt,
    };

    // Add each control scheme to the menu. Each control scheme has a submenu to select difficulty
    for (uint8_t i = 0; i < ARRAY_SIZE(controlSchemes); i++)
    {
        // Add a control scheme to the menu. This opens a submenu with difficulties
        maze->menu = startSubMenu(maze->menu, controlSchemes[i]);
        // Add difficulties to the submenu
        addSingleItemToMenu(maze->menu, mazeDiffEasy);
        addSingleItemToMenu(maze->menu, mazeDiffMedium);
        addSingleItemToMenu(maze->menu, mazeDiffHard);
        // End the submenu
        maze->menu = endSubMenu(maze->menu);
    }

    // Set the mode to menu mode
    maze->screen = MAZE_MENU;
}

/**
 * This function is called when the mode is exited. It deinitializes variables and frees all memory.
 */
static void mazeExitMode(void)
{
    // Deinitialize the menu
    deinitMenu(maze->menu);
    deinitMenuLogbookRenderer(maze->menuLogbookRenderer);
    // Free the font
    freeFont(&maze->ibm);
    // Free graphics
    freeWsg(&maze->ballWsg);
    // Free the songs
    freeSong(&maze->bgm);
    freeSong(&maze->hit1);
    // Free everything else
    free(maze);
}

/**
 * @brief This callback function is called when an item is selected from the menu
 *
 * @param label The item that was selected from the menu
 * @param selected True if the item was selected with the A button, false if this is a multi-item which scrolled to
 * @param settingVal The value of the setting, if the menu item is a settings item
 */
static void mazeMenuCb(const char* label, bool selected, uint32_t settingVal)
{
    // Only care about selected items, not scrolled-to items.
    // The same callback is called from the menu and submenu with no indication of which menu it was called from
    // Note that the label arg will be one of the strings used in startSubMenu() or addSingleItemToMenu()
    if (selected)
    {
        // Save what control scheme is selected (first-level menu)
        if (label == mazeCtrlButton)
        {
            maze->control = MAZE_BUTTON;
        }
        // Save what control scheme is selected (first-level menu)
        else if (label == mazeCtrlTouch)
        {
            maze->control = MAZE_TOUCH;
        }
        // Save what control scheme is selected (first-level menu)
        else if (label == mazeCtrlTilt)
        {
            maze->control = MAZE_TILT;
        }
        // Save what difficulty is selected and start the game (second-level menu)
        else if (label == mazeDiffEasy)
        {
            maze->difficulty = MAZE_EASY;
            mazeResetGame();
            maze->screen = MAZE_GAME;
        }
        // Save what difficulty is selected and start the game (second-level menu)
        else if (label == mazeDiffMedium)
        {
            maze->difficulty = MAZE_MEDIUM;
            mazeResetGame();
            maze->screen = MAZE_GAME;
        }
        // Save what difficulty is selected and start the game (second-level menu)
        else if (label == mazeDiffHard)
        {
            maze->difficulty = MAZE_HARD;
            mazeResetGame();
            maze->screen = MAZE_GAME;
        }
    }
}

/**
 * @brief This function is called periodically and frequently. It will either draw the menu or play the game, depending
 * on which screen is currently being displayed
 *
 * @param elapsedUs The time that has elapsed since the last call to this function, in microseconds
 */
static void mazeMainLoop(int64_t elapsedUs)
{
    // Pick what runs and draws depending on the screen being displayed
    switch (maze->screen)
    {
        case MAZE_MENU:
        {
            // Process button events
            buttonEvt_t evt = {0};
            while (checkButtonQueueWrapper(&evt))
            {
                // Pass button events to the menu
                maze->menu = menuButton(maze->menu, evt);
            }

            // Draw the menu
            drawMenuLogbook(maze->menu, maze->menuLogbookRenderer, elapsedUs);
            break;
        }
        case MAZE_GAME:
        {
            // Run the main game loop. This will also process button events
            mazeGameLoop(elapsedUs);
            break;
        }
    }
}

/**
 * @brief This function is called periodically and frequently. It runs the actual game, including processing inputs,
 * physics updates and drawing to the display.
 *
 * @param elapsedUs The time that has elapsed since the last call to this function, in microseconds
 */
static void mazeGameLoop(int64_t elapsedUs)
{
    // Always process button events, regardless of control scheme, so the main menu button can be captured
    buttonEvt_t evt = {0};
    while (checkButtonQueueWrapper(&evt))
    {
        // Save the button state
        maze->btnState = evt.state;

        // Check if the pause button was pressed
        if (evt.down && (PB_START == evt.button))
        {
            // Toggle pause
            maze->isPaused = !maze->isPaused;
        }
    }

    // If the game is paused
    if (maze->isPaused)
    {
        // Just draw and return
        mazeDrawField();
        return;
    }

    // While the restart timer is active
    while (maze->restartTimerUs > 0)
    {
        // Decrement the timer and draw the field, but don't run game logic
        maze->restartTimerUs -= elapsedUs;
        mazeDrawField();
        return;
    }

    // Do update each loop
    mazeHandlePlayerInput();
    mazeUpdatePhysics(elapsedUs);

    // Draw the field
    mazeDrawField();
}

/**
 * @brief Move the player's paddle according to the chosen control scheme
 */
static void mazeHandlePlayerInput(void)
{
    // Move the paddle depending on the chosen control scheme
    switch (maze->control)
    {
        default:
        case MAZE_BUTTON:
        {
            // Move the player paddle if a button is currently down
            if (maze->btnState & PB_UP)
            {
                ESP_LOGI("Maze", "UP");
                maze->tiltV += BUTTON_INCREMENT;
            }
            else if (maze->btnState & PB_DOWN)
            {
                ESP_LOGI("Maze", "DOWN");
                maze->tiltV -= BUTTON_INCREMENT;
            }

            if (maze->btnState & PB_LEFT)
            {
                ESP_LOGI("Maze", "LEFT");
                maze->tiltH += BUTTON_INCREMENT;
            }
            else if (maze->btnState & PB_RIGHT)
            {
                ESP_LOGI("Maze", "RIGHT");
                maze->tiltH -= BUTTON_INCREMENT;
            }
            break;
        }
        case MAZE_TOUCH:
        {
            // TODO
            break;
        }
        case MAZE_TILT:
        {
            // Declare variables to receive acceleration
            int16_t a_x, a_y, a_z;
            // Get the current acceleration
            if (ESP_OK == accelGetAccelVec(&a_x, &a_y, &a_z))
            {
                // ok, so acceleration is a vector
                // we assume a^2+b^2+c^2 == (9.8)^2 m/s^2s
                maze->tiltH = a_y * 360 / 242;
                maze->tiltV = a_x * 10;
            }
            break;
        }
    }

    maze->tiltH = CLAMP(maze->tiltH, -9000, 9000);
    maze->tiltV = CLAMP(maze->tiltV, -9000, 9000);

    if (maze->btnState & PB_START)
    {
        mazeResetGame();
    }
}

/**
 * @brief Update the maze physics including ball position and collisions
 *
 * @param elapsedUs The time that has elapsed since the last call to this function, in microseconds
 */
static void mazeUpdatePhysics(int64_t elapsedUs)
{
    // Calculate normal forces for ball in each direction
    // Do math, apply acceleration
    // Apply velocity / update the ball's position
    // Do collision checks
    // Do goal check

    // First: move the ball according to its current velocity

    maze->ball.x += (maze->ballVel.x * elapsedUs) / 100000;
    maze->ball.y += (maze->ballVel.y * elapsedUs) / 100000;

    // Then, we calculate the velocity

    // Convert the tilt from "degrees off-from-vertical" to actual trig degrees
    // From that, we can use sine to get the proportion of force to apply to movement,
    // and in which direction.
    // TODO: fact check these next two lines
    // For H (left-to-right tilt), negative moves the ball to the right and positive to the left
    // For V (top-to-bottom tilt), negative moves the ball to the bottom and positive to the top
    uint16_t realTiltH = (36000 + maze->tiltH) % 36000;
    uint16_t realTiltV = (36000 + maze->tiltV) % 36000;

    ESP_LOGI("Maze", "Tilt(H,V) = (%" PRId16 ".%" PRId16 ", %" PRId16 ".%" PRId16 ")", (realTiltH / 100),
             realTiltH % 100, realTiltV / 100, realTiltV % 100);

    vec_t normalForce;

    // F = Ma
    // F = M(g) * a(cm/s/s)
    // F(g)(cm/s^2) --> kgm/s^2 = *(1kg/1000g)*(1m/100cm) = *(1 / 100,000) --> (1,000mN / 1N) --> (1,000 / 100,000) -->
    // (1 / 100) Divide by 100 to get milli-newtons
    normalForce.x = getSin1024(realTiltV / 100) * GRAVITY;
    normalForce.y = getSin1024(realTiltH / 100) * GRAVITY;
    // ESP_LOGI("Maze", "normalForce = (%" PRId32 ", %" PRId32 ")", normalForce.x, normalForce.y);

    vec_t newAccel = divVec2d(normalForce, maze->puzzle.ballMass * 1024);
    maze->ballVel  = addVec2d(maze->ballVel, divVec2d(mulVec2d(newAccel, elapsedUs), 100000));
}

/**
 * @brief Reset the maze game variables
 */
static void mazeResetGame(void)
{
    // Set different variables based on initialization
    maze->puzzle.startX       = TFT_WIDTH / 2;
    maze->puzzle.startY       = TFT_HEIGHT / 2;
    maze->ball.x              = maze->puzzle.startX;
    maze->ball.y              = maze->puzzle.startY;
    maze->ball.radius         = BALL_RADIUS;
    maze->puzzle.ballMass     = 50; // 50g ball
    maze->puzzle.ballFriction = 1;  // 1% friction

    // Start playing music
    bzrPlayBgm(&maze->bgm);

    // Set the restart timer
    maze->restartTimerUs = 2000000;

    // Reset the ball's velocity to 0
    maze->ballVel.x = 0;
    maze->ballVel.y = 0;

    maze->tiltH = 0;
    maze->tiltV = 0;
}

/**
 * This function is called when the display driver wishes to update a
 * section of the display.
 *
 * @param x the x coordinate that should be updated
 * @param y the x coordinate that should be updated
 * @param w the width of the rectangle to be updated
 * @param h the height of the rectangle to be updated
 * @param up update number
 * @param numUp update number denominator
 */
static void mazeBackgroundDrawCallback(int16_t x, int16_t y, int16_t w, int16_t h, int16_t up, int16_t upNum)
{
    // Use TURBO drawing mode to draw individual pixels fast
    SETUP_FOR_TURBO();

    // Draw a grid
    for (int16_t yp = y; yp < y + h; yp++)
    {
        for (int16_t xp = x; xp < x + w; xp++)
        {
            if ((0 == xp % 40) || (0 == yp % 40))
            {
                TURBO_SET_PIXEL(xp, yp, c110);
            }
            else
            {
                TURBO_SET_PIXEL(xp, yp, c001);
            }
        }
    }
}

/**
 * @brief Draw the Maze field to the TFT
 */
static void mazeDrawField(void)
{
    // Create an array for all LEDs, all 0
    led_t leds[CONFIG_NUM_LEDS] = {0};

    // Set the LED output
    setLeds(leds, CONFIG_NUM_LEDS);

    char posStr[32] = {0};
    snprintf(posStr, sizeof(posStr) - 1, "(%02" PRId32 ", %02" PRId32 ")", maze->ball.x, maze->ball.y);
    drawText(&maze->ibm, c555, posStr, 15, 15);
    snprintf(posStr, sizeof(posStr) - 1, "H,V=(%02" PRId16 ", %02" PRId16 ")", maze->tiltH, maze->tiltV);
    drawText(&maze->ibm, c555, posStr, 15, 15 + maze->ibm.height + 1);

    // Draw the ball
    drawWsgSimple(&maze->ballWsg, (maze->ball.x - maze->ball.radius),
                  (maze->ball.y - maze->ball.radius));

    // Set up variables to draw text
    char scoreStr[16] = {0};
    int16_t tWidth;

    // If the restart timer is active, draw it
    if (maze->isPaused)
    {
        // Measure the width of the time string
        tWidth = textWidth(&maze->ibm, mazePaused);
        // Draw the time string to the display, centered at (TFT_WIDTH / 2)
        drawText(&maze->ibm, c555, mazePaused, ((TFT_WIDTH - tWidth) / 2), 0);
    }
    else if (maze->restartTimerUs > 0)
    {
        // Render the time to a string
        snprintf(scoreStr, sizeof(scoreStr) - 1, "%01" PRId32 ".%03" PRId32, maze->restartTimerUs / 1000000,
                 (maze->restartTimerUs / 1000) % 1000);
        // Measure the width of the time string
        tWidth = textWidth(&maze->ibm, scoreStr);
        // Draw the time string to the display, centered at (TFT_WIDTH / 2)
        drawText(&maze->ibm, c555, scoreStr, ((TFT_WIDTH - tWidth) / 2), 0);
    }
}

/**
 * This function is called whenever an ESP-NOW packet is received.
 *
 * @param esp_now_info Information about the transmission, including The MAC addresses
 * @param data A pointer to the data received
 * @param len The length of the data received
 * @param rssi The RSSI for this packet, from 1 (weak) to ~90 (touching)
 */
static void mazeEspNowRecvCb(const esp_now_recv_info_t* esp_now_info, const uint8_t* data, uint8_t len, int8_t rssi)
{
    p2pRecvCb(&maze->p2p, esp_now_info->src_addr, data, len, rssi);
}

/**
 * This function is called whenever an ESP-NOW packet is sent.
 * It is just a status callback whether or not the packet was actually sent.
 * This will be called after calling espNowSend()
 *
 * @param mac_addr The MAC address which the data was sent to
 * @param status The status of the transmission
 */
static void mazeEspNowSendCb(const uint8_t* mac_addr, esp_now_send_status_t status)
{
    p2pSendCb(&maze->p2p, mac_addr, status);
}
