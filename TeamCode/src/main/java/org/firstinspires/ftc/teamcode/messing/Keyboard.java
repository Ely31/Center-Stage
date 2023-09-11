package org.firstinspires.ftc.teamcode.messing;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.REDetector;

public class Keyboard {

    final int width = 11;
    final int height = 3;

    int row = 0;
    int col = 0;

    public int getRow() {
        return row;
    }
    public int getCol() {
        return col;
    }

    public void shiftRow(int shiftAmount){
        row += shiftAmount;
        if (row > height) row = 0;
        if (row < 0) row = height;
    }
    public void shiftCol(int shiftAmount){
        col += shiftAmount;
        if (col > width) col = 0;
        if (col < 0) col = width;
    }

    /* Keymap declaration
    The layout is wierd in some places because a normal keyboard doesn't have the same
    number of keys on each row, but this one does. */
    final char[] l0row0 = {'1','2','3','4','5','6','7','8','9','0','-','='};
    final char[] l0row1 = {'q','w','f','p','g','j','l','u','y',';','[',']'};
    final char[] l0row2 = {'a','r','s','t','d','h','n','e','i','o','\'','\\'};
    final char[] l0row3 = {'z','x','c','v','b','k','m',',','.','/','?','!'};
    final char[][] layer0 = {l0row0, l0row1, l0row2, l0row3};

    final char[] l1row0 = {'!','@','#','$','%','^','&','*','(',')','_','+'};
    final char[] l1row1 = {'Q','W','F','P','G','J','L','U','Y',':','{','}'};
    final char[] l1row2 = {'A','R','S','T','D','H','N','E','I','O','"','|'};
    final char[] l1row3 = {'Z','X','C','V','B','K','M','<','>','/','`','~'};
    final char[][] layer1 = {l1row0, l1row1, l1row2, l1row3};

    // This keymap can be changed if you press shift and is the one all inputs are checked on
    char[][] keymap = layer0;

    // This enables the behavior of pressing the shift key
    public void setLayer(int layer){
        switch (layer){
            case 0:
                keymap = layer0;
                break;
            case 1:
                keymap = layer1;
                break;
            default:
                keymap = layer0;
        }
    }

    public String getDisplayString(){
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i <= height; i++) {
            for (int j = 0; j <= width; j++) {
                // If this iterates over the key you have selected, display it as a solid box instead of its character
                if (i == row && j == col){
                    builder.append('█');
                } else {
                    builder.append(keymap[i][j]);
                }
            }
            if (i < height) builder.append('\n');
        }
        return builder.toString();
    }

    public char getSelectedChar(){
        return keymap[row][col];
    }

    // Gamepad interaction stuff
    REDetector navLeft = new REDetector();
    REDetector navRight = new REDetector();
    REDetector navDown = new REDetector();
    REDetector navUp = new REDetector();

    REDetector typeLetter = new REDetector();
    REDetector space = new REDetector();
    REDetector del = new REDetector();
    REDetector enter = new REDetector();

    Gamepad gamepad;

    public Keyboard(Gamepad gamepad){
        this.gamepad = gamepad;
    }

    public void update() {
        // Move the selected key around
        if (navLeft.status(gamepad.dpad_left)) shiftCol(-1);
        if (navRight.status(gamepad.dpad_right)) shiftCol(1);
        if (navDown.status(gamepad.dpad_down)) shiftRow(1);
        if (navUp.status(gamepad.dpad_up)) shiftRow(-1);

        if (shiftPressed()) setLayer(1);
        else setLayer(0);
    }

    public boolean letterTyped(){
        return typeLetter.status(gamepad.cross);
    }
    public boolean spacePressed(){
        return space.status(gamepad.triangle);
    }
    public boolean delPressed(){
        return del.status(gamepad.square);
    }
    public boolean enterPressed(){
        return enter.status(gamepad.circle);
    }
    public boolean shiftPressed(){
        return gamepad.left_bumper;
    }
}
