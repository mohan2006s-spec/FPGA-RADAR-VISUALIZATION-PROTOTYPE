module radar_top (
    input CLOCK_50,
    input [7:0] SW,
    input [1:0] KEY,
    output VGA_HS,
    output VGA_VS,
    output [3:0] VGA_R,
    output [3:0] VGA_G,
    output [3:0] VGA_B,
    output [7:0] LED
);

    // 1. Pixel clock: 50 MHz / 2
    reg pixel_clk = 1'b0;

    always @(posedge CLOCK_50)
        pixel_clk <= ~pixel_clk;

    // 2. VGA counters
    reg [9:0] h_count = 0;
    reg [9:0] v_count = 0module radar_top (
    input CLOCK_50,
    input [7:0] SW,
    input [1:0] KEY,
    output VGA_HS,
    output VGA_VS,
    output [3:0] VGA_R,
    output [3:0] VGA_G,
    output [3:0] VGA_B,
    output [7:0] LED
);

    // 1. Pixel clock: 50 MHz / 2
    reg pixel_clk = 1'b0;

    always @(posedge CLOCK_50)
        pixel_clk <= ~pixel_clk;

    // 2. VGA counters
    reg [9:0] h_count = 0;
    reg [9:0] v_count = 0;

    always @(posedge pixel_clk or negedge KEY[0]) begin
        if (!KEY[0]) begin
            h_count <= 0;
            v_count <= 0;
        end else begin
            if (h_count == 799) begin
                h_count <= 0;
                if (v_count == 524)
                    v_count <= 0;
                else
                    v_count <= v_count + 1'b1;
            end else begin
                h_count <= h_count + 1'b1;
            end
        end
    end

    // 3. VGA synchronization
    assign VGA_HS = ~((h_count >= 656) &&
                      (h_count < 752));

    assign VGA_VS = ~((v_count >= 490) &&
                      (v_count < 492));

    wire display_on = (h_count < 640) &&
                      (v_count < 480);

    // 4. Radar centre
    wire signed [10:0] dx, dy;

    assign dx = $signed({1'b0, h_count}) - 11'sd320;
    assign dy = 11'sd300 - $signed({1'b0, v_count});

    wire signed [21:0] dx_sq, dy_sq;
    wire [21:0] distance_sq;

    assign dx_sq = dx * dx;
    assign dy_sq = dy * dy;
    assign distance_sq = dx_sq + dy_sq;

    wire radar_circle;

    assign radar_circle =
        (distance_sq >= 22'd19000) &&
        (distance_sq <= 22'd21000);

    // 5. VERY SLOW radar rotation
    // 32 steps x 0.5 second = 16 seconds per rotation
    reg [4:0] angle = 0;
    reg [25:0] scan_timer = 0;

    always @(posedge CLOCK_50 or negedge KEY[0]) begin
        if (!KEY[0]) begin
            angle <= 0;
            scan_timer <= 0;
        end else begin
            if (scan_timer >= 26'd24999999) begin
                scan_timer <= 0;
                angle <= angle + 1'b1;
            end else begin
                scan_timer <= scan_timer + 1'b1;
            end
        end
    end

    // 6. Scan endpoint lookup table
    reg signed [8:0] scan_ex, scan_ey;

    always @(*) begin
        case (angle)
            0:  begin scan_ex=140;  scan_ey=0;    end
            1:  begin scan_ex=137;  scan_ey=27;   end
            2:  begin scan_ex=129;  scan_ey=54;   end
            3:  begin scan_ex=116;  scan_ey=79;   end
            4:  begin scan_ex=99;   scan_ey=99;   end
            5:  begin scan_ex=79;   scan_ey=116;  end
            6:  begin scan_ex=54;   scan_ey=129;  end
            7:  begin scan_ex=27;   scan_ey=137;  end
            8:  begin scan_ex=0;    scan_ey=140;   end
            9:  begin scan_ex=-27;  scan_ey=137;  end
            10: begin scan_ex=-54;  scan_ey=129;  end
            11: begin scan_ex=-79;  scan_ey=116;  end
            12: begin scan_ex=-99;  scan_ey=99;   end
            13: begin scan_ex=-116; scan_ey=79;   end
            14: begin scan_ex=-129; scan_ey=54;   end
            15: begin scan_ex=-137; scan_ey=27;   end
            16: begin scan_ex=-140; scan_ey=0;    end
            17: begin scan_ex=-137; scan_ey=-27;  end
            18: begin scan_ex=-129; scan_ey=-54;  end
            19: begin scan_ex=-116; scan_ey=-79;  end
            20: begin scan_ex=-99;  scan_ey=-99;  end
            21: begin scan_ex=-79;  scan_ey=-116; end
            22: begin scan_ex=-54;  scan_ey=-129; end
            23: begin scan_ex=-27;  scan_ey=-137; end
            24: begin scan_ex=0;    scan_ey=-140; end
            25: begin scan_ex=27;   scan_ey=-137; end
            26: begin scan_ex=54;   scan_ey=-129; end
            27: begin scan_ex=79;   scan_ey=-116; end
            28: begin scan_ex=99;   scan_ey=-99;  end
            29: begin scan_ex=116;  scan_ey=-79;  end
            30: begin scan_ex=129;  scan_ey=-54;  end
            31: begin scan_ex=137;  scan_ey=-27;  end
            default: begin scan_ex=140; scan_ey=0; end
        endcase
    end

    // 7. Scan line coordinates
    wire signed [10:0] scan_x_s, scan_y_s;

    assign scan_x_s = 11'sd320 + scan_ex;
    assign scan_y_s = 11'sd300 - scan_ey;

    wire signed [20:0] cross_product;
    wire signed [20:0] dot_product;
    wire [20:0] cross_abs;

    assign cross_product = dx * scan_ey - dy * scan_ex;
    assign dot_product = dx * scan_ex + dy * scan_ey;

    assign cross_abs = cross_product[20]
                     ? -cross_product : cross_product;

    wire scan_line;

    assign scan_line =
        (cross_abs <= 21'd140) &&
        (dot_product >= 0) &&
        (dot_product <= 21'd19600);

    // 8. Scan endpoint
    wire scan_point;

    assign scan_point =
        (h_count >= scan_x_s - 11'sd2) &&
        (h_count <= scan_x_s + 11'sd2) &&
        (v_count >= scan_y_s - 11'sd2) &&
        (v_count <= scan_y_s + 11'sd2);

    // 9. Target positions controlled by switches
    wire [9:0] target1_x, target1_y;
    wire [9:0] target2_x, target2_y;
    wire [9:0] target3_x, target3_y;

    assign target1_x = 10'd220 + (SW[1:0] * 10'd20);
    assign target1_y = 10'd320;

    assign target2_x = 10'd240 + (SW[3:2] * 10'd20);
    assign target2_y = 10'd260;

    assign target3_x = 10'd300 + (SW[5:4] * 10'd10);
    assign target3_y = 10'd350;

    // 10. Target pixels
    wire target1, target2, target3;

    assign target1 =
        (h_count >= target1_x - 10'd3) &&
        (h_count <= target1_x + 10'd3) &&
        (v_count >= target1_y - 10'd3) &&
        (v_count <= target1_y + 10'd3);

    assign target2 =
        (h_count >= target2_x - 10'd3) &&
        (h_count <= target2_x + 10'd3) &&
        (v_count >= target2_y - 10'd3) &&
        (v_count <= target2_y + 10'd3);

    assign target3 =
        (h_count >= target3_x - 10'd3) &&
        (h_count <= target3_x + 10'd3) &&
        (v_count >= target3_y - 10'd3) &&
        (v_count <= target3_y + 10'd3);

    // 11. Radar crosshair
    wire radar_crosshair;

    assign radar_crosshair =
        ((h_count >= 10'd315) &&
         (h_count <= 10'd325) &&
         (v_count >= 10'd160) &&
         (v_count <= 10'd300))
        ||
        ((v_count >= 10'd295) &&
         (v_count <= 10'd305) &&
         (h_count >= 10'd180) &&
         (h_count <= 10'd460));

    // 12. VGA colours
    reg [3:0] red, green, blue;

    always @(*) begin
        red   = 4'h0;
        green = 4'h0;
        blue  = 4'h0;

        if (display_on) begin
            if (target1 || target2 || target3) begin
                red = 4'hF;
            end else if (scan_point) begin
                red   = 4'hF;
                green = 4'hF;
            end else if (scan_line) begin
                green = 4'hF;
            end else if (radar_circle || radar_crosshair) begin
                green = 4'hF;
            end
        end
    end

    assign VGA_R = red;
    assign VGA_G = green;
    assign VGA_B = blue;

    // LEDs display scan angle
    assign LED = {3'b000, angle};

endmodule;

    always @(posedge pixel_clk or negedge KEY[0]) begin
        if (!KEY[0]) begin
            h_count <= 0;
            v_count <= 0;
        end else begin
            if (h_count == 799) begin
                h_count <= 0;
                if (v_count == 524)
                    v_count <= 0;
                else
                    v_count <= v_count + 1'b1;
            end else begin
                h_count <= h_count + 1'b1;
            end
        end
    end

    // 3. VGA synchronization
    assign VGA_HS = ~((h_count >= 656) &&
                      (h_count < 752));

    assign VGA_VS = ~((v_count >= 490) &&
                      (v_count < 492));

    wire display_on = (h_count < 640) &&
                      (v_count < 480);

    // 4. Radar centre
    wire signed [10:0] dx, dy;

    assign dx = $signed({1'b0, h_count}) - 11'sd320;
    assign dy = 11'sd300 - $signed({1'b0, v_count});

    wire signed [21:0] dx_sq, dy_sq;
    wire [21:0] distance_sq;

    assign dx_sq = dx * dx;
    assign dy_sq = dy * dy;
    assign distance_sq = dx_sq + dy_sq;

    wire radar_circle;

    assign radar_circle =
        (distance_sq >= 22'd19000) &&
        (distance_sq <= 22'd21000);

    // 5. VERY SLOW radar rotation
    // 32 steps x 0.5 second = 16 seconds per rotation
    reg [4:0] angle = 0;
    reg [25:0] scan_timer = 0;

    always @(posedge CLOCK_50 or negedge KEY[0]) begin
        if (!KEY[0]) begin
            angle <= 0;
            scan_timer <= 0;
        end else begin
            if (scan_timer >= 26'd24999999) begin
                scan_timer <= 0;
                angle <= angle + 1'b1;
            end else begin
                scan_timer <= scan_timer + 1'b1;
            end
        end
    end

    // 6. Scan endpoint lookup table
    reg signed [8:0] scan_ex, scan_ey;

    always @(*) begin
        case (angle)
            0:  begin scan_ex=140;  scan_ey=0;    end
            1:  begin scan_ex=137;  scan_ey=27;   end
            2:  begin scan_ex=129;  scan_ey=54;   end
            3:  begin scan_ex=116;  scan_ey=79;   end
            4:  begin scan_ex=99;   scan_ey=99;   end
            5:  begin scan_ex=79;   scan_ey=116;  end
            6:  begin scan_ex=54;   scan_ey=129;  end
            7:  begin scan_ex=27;   scan_ey=137;  end
            8:  begin scan_ex=0;    scan_ey=140;   end
            9:  begin scan_ex=-27;  scan_ey=137;  end
            10: begin scan_ex=-54;  scan_ey=129;  end
            11: begin scan_ex=-79;  scan_ey=116;  end
            12: begin scan_ex=-99;  scan_ey=99;   end
            13: begin scan_ex=-116; scan_ey=79;   end
            14: begin scan_ex=-129; scan_ey=54;   end
            15: begin scan_ex=-137; scan_ey=27;   end
            16: begin scan_ex=-140; scan_ey=0;    end
            17: begin scan_ex=-137; scan_ey=-27;  end
            18: begin scan_ex=-129; scan_ey=-54;  end
            19: begin scan_ex=-116; scan_ey=-79;  end
            20: begin scan_ex=-99;  scan_ey=-99;  end
            21: begin scan_ex=-79;  scan_ey=-116; end
            22: begin scan_ex=-54;  scan_ey=-129; end
            23: begin scan_ex=-27;  scan_ey=-137; end
            24: begin scan_ex=0;    scan_ey=-140; end
            25: begin scan_ex=27;   scan_ey=-137; end
            26: begin scan_ex=54;   scan_ey=-129; end
            27: begin scan_ex=79;   scan_ey=-116; end
            28: begin scan_ex=99;   scan_ey=-99;  end
            29: begin scan_ex=116;  scan_ey=-79;  end
            30: begin scan_ex=129;  scan_ey=-54;  end
            31: begin scan_ex=137;  scan_ey=-27;  end
            default: begin scan_ex=140; scan_ey=0; end
        endcase
    end

  
