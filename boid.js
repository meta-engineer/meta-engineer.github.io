class boid {
    constructor(x, y, vx=0, vy=0, mv = 400) {
        this.x = x;
        this.y = y;
        this.vx = vx; // velocities are units/1000ms
        this.vy = vy;
        this.maxV = mv; //mv is maintained in the case of no primeupdate
        this.ax = 0;
        this.ay = 0;
        this.cos0 = 1;
        this.sin0 = 0;

        this.drawSz = 16; //drawSz is maintained for bounds check
    }
//(this.boids, this.maxV, this.viewR, this.viewR2, this.viewA, this.alignCoef, this.separateCoef, this.cohesionCoef);
    primeUpdate(group, mv, vr, vr2, vAng, aCoef, sCoef, cCoef) {
        // update accel based on the flock
        // updates are grouped to reduce looping over the flock
        // variables: Separation, Alignment, Cohesion
        // independant variables: view distance, change magnitude
        // separation: get vector to all boid in view, sum, inverse (stronger farther away)
        // alignment: get velocity vector of all void in view, average
        // cohesion: get postion of all boid in view, avg, vector from me to avg (stronger further away)

        //update this
        this.maxV = mv;

        let viewed = 0;
        let v2ewed = 0;
        let sep = { x:0, y:0 };
        let ali = { x:0, y:0 };
        let coh = { x:0, y:0 };

        // assume flock is sorted based on x (from drawCanvas) and try to exit early
        for (let i = 0; i < group.length; i++) {
            let b = group[i];
            // skip boids in early slices, short-circuit once passed
            if (b == this) continue;
            if (b.x < this.x - vr) continue;
            if (b.x > this.x + vr) break;
            // limited vision: exit early if b is behind this by certain angle, 45deg?
            if (vAng <= -1) continue; // easy answer
            if (vAng < 1) {     // only do hypots if nessisary
                let towardB = { x: b.x - this.x, y: b.y - this.y };
                let dp = towardB.x*this.vx + towardB.y*this.vy;
                
                let tBMag = Math.hypot(towardB.x, towardB.y);
                let vMag = Math.hypot(this.vx, this.vy);
                if ((dp / vMag) / tBMag < vAng) {
                    continue;
                }
            }
            
           //if (dp < -0.5) continue;

            // within radius (r2 calculation to optimize)
            if ((b.x - this.x)**2 + (b.y - this.y)**2 <= vr2) {
                viewed += 1;
                coh.x += b.x;
                coh.y += b.y;
            }
            // separation and alignment apply when closer
            if ((b.x - this.x)**2 + (b.y - this.y)**2 <= vr2/2.0) {
                v2ewed += 1;
                let d2 = (this.x - b.x)**2 + (this.y - b.y)**2;
                sep.x += (this.x - b.x) / d2;
                sep.y += (this.y - b.y) / d2;
                ali.x += b.vx;
                ali.y += b.vy;
            }
        }

        if (viewed == 0) return;
        // Apply cohesion to acceleration
        // get average
        coh.x /= viewed;
        coh.y /= viewed;
        // get vector from me to avg
        coh.x -= this.x;
        coh.y -= this.y;
        // "normalize"
        let cMag = Math.sqrt(coh.x**2 + coh.y**2);
        if (cMag) {
            coh.x *= (this.maxV/cMag) * cCoef;
            coh.y *= (this.maxV/cMag) * cCoef;
        }
        // "steering" vector
        if (coh.x && coh.y) {
            this.ax += (coh.x - this.vx);
            this.ay += (coh.y - this.vy);
        }

        if (v2ewed == 0) return;
        // Apply separation
        // avarage and normalize?
        sep.x /= v2ewed;
        sep.y /= v2ewed;
        // "normalize"
        let sMag = Math.sqrt(sep.x**2 + sep.y**2);
        if (sMag) {
            sep.x *= (this.maxV/sMag) * sCoef; 
            sep.y *= (this.maxV/sMag) * sCoef;
        }
        // add "steering" vector
        if (sep.x && sep.y) {
            this.ax += (sep.x - this.vx);
            this.ay += (sep.y - this.vy);
        }

        // Apply alignment
        ali.x /= v2ewed;
        ali.y /= v2ewed;
        // "normalize"
        let aMag = Math.sqrt(ali.x**2 + ali.y**2);
        if (aMag) {
            ali.x *= (this.maxV/aMag) * aCoef;
            ali.y *= (this.maxV/aMag) * aCoef;
        }
        // "steering" vector
        if (ali.x && ali.y) {   // NaN check
            this.ax += (ali.x - this.vx);
            this.ay += (ali.y - this.vy);
        }
    }

    update(bounds, ts) {
        if (!this.ax) this.ax = 0;
        if (!this.ay) this.ay = 0;

        this.vx += this.ax * (ts/1000);
        this.vy += this.ay * (ts/1000);
        this.x += this.vx * (ts/1000);
        this.y += this.vy * (ts/1000);
        this.ax = 0;
        this.ay = 0;

        // this requires normalisation which is expensive so for now...
        if (this.vx > this.maxV) this.vx = this.maxV;
        if (this.vx <-this.maxV) this.vx = -this.maxV;
        if (this.vy > this.maxV) this.vy = this.maxV;
        if (this.vy <-this.maxV) this.vy = -this.maxV;

        // wrap or collision?
        // view buffer, tied to size of shape?
        // As boids are wrapped there is an artifact
        
        // wrap
        let mw = bounds.width + this.drawSz;
        let mh = bounds.height + this.drawSz;
        if (this.x < -this.drawSz) this.x += mw;
        if (this.x > mw)            this.x %= mw;
        if (this.y < -this.drawSz) this.y += mh;
        if (this.y > mh)            this.y %= mh;
        
    }

    drawSelf(ctx) {
        // care for x=0
        if (this.vx == 0 && this.vy == 0) {
            // just use previous stored cos/sin
        } else {
            // vector rotation with another vector
            let h = Math.hypot(this.vx, this.vy);
            this.cos0 = this.vx / h;
            this.sin0 = this.vy / h;
        }
        // first point is -5,20 second point is 5,20
        let x1 = this.cos0 * (-this.drawSz) - this.sin0 * (-this.drawSz/4);
        let y1 = this.sin0 * (-this.drawSz) + this.cos0 * (-this.drawSz/4);
        let x2 = this.cos0 * (-this.drawSz) - this.sin0 * ( this.drawSz/4);
        let y2 = this.sin0 * (-this.drawSz) + this.cos0 * ( this.drawSz/4);

        ctx.beginPath();
        ctx.lineWidth = "2";
        ctx.strokeStyle = "white";
        ctx.moveTo(this.x, this.y);
        ctx.lineTo(this.x + x1, this.y + y1);
        ctx.lineTo(this.x + x2, this.y + y2);
        ctx.lineTo(this.x, this.y);
        ctx.stroke(); // batch stroking after drawing?
    }
}