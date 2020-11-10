class flock {
    constructor(pop, bw, bh, mv=400) {
        this.pop = pop;
        this.bounds = {width:bw, height:bh};
        this.maxV = mv;

        // if boid attributes are stored here they are passed to boids each primeUpdate. this removes individuality of each boid and asserts all boids have identical traits.
        // do we want/need a boid with multiple different attributes?
        this.viewR = 160;
        this.viewR2 = this.viewR**2;
        this.viewA = -0.3;
        this.alignCoef = 1.0;
        this.separateCoef = 1.0;
        this.cohesionCoef = 0.9;
        
        //this.drawSz = 16;
        
        this.boids = new Array();
        for (let i = 0; i < this.pop; i++) {
            this.boids.push(new boid(Math.random() * this.bounds.width, Math.random() * this.bounds.height, Math.random()*40 - 20, Math.random()*40 - 20, this.maxV));
        }
        
    }

    sortX() {
        this.boids.sort((a,b) => {
            return a.x - b.x;
        });
    }

    primeUpdate() {
        this.boids.forEach((b) => {
            b.primeUpdate(this.boids, this.maxV, this.viewR, this.viewR2, this.viewA, this.alignCoef, this.separateCoef, this.cohesionCoef);
        });
    }

    update(timeStep) {
        this.boids.forEach((b) => {
            b.update(this.bounds, timeStep);
        });
    }

    draw(ctx) {
        this.boids.forEach((b) => {
            b.drawSelf(ctx);
        });
    }

    applyRelativeForce(pos, rad, mag) {
        for (let i = 0; i < this.boids.length; i++) {
            if (this.boids[i].x < pos.x - rad) continue;
            if (this.boids[i].x > pos.x + rad) break;
            // apply force from boid to end of radius, through mousePos
            let forceVec = { x: this.boids[i].x - pos.x, 
                y: this.boids[i].y - pos.y };
            let dist = Math.hypot(forceVec.x, forceVec.y);
            if (dist > rad) continue;
            forceVec.x /= dist;
            forceVec.y /= dist;
            forceVec.x *= rad - dist;
            forceVec.y *= rad - dist;
            forceVec.x *= mag;
            forceVec.y *= mag;
            this.boids[i].ax += forceVec.x;
            this.boids[i].ay += forceVec.y;
        }
    }

    setBounds(bw, bh) {
        this.bounds.width = bw;
        this.bounds.height= bh;
    }

    setMaxV(v) {
        this.maxV = v;
    }

    // will this cause concurrency errors???
    setPop(p) {
        console.log(p);
        if (p < 0 || p > 2000) return;
        if (this.pop < p) {
            while (this.boids.length < p) {
                this.boids.push(new boid(Math.random() * this.bounds.width, Math.random() * this.bounds.height, Math.random()*40 - 20, Math.random()*40 - 20, 160, this.maxV));
            }
        }
        if (this.pop > p) {
            while (this.boids.length > p) {
                this.boids.splice(Math.floor(Math.random()*this.boids.length), 1);
            }
        }
        this.pop = p; // dont wait for garbage day to stop drawing!
    }

    setRadius(r) {
        this.viewR = r;
        this.viewR2 = r*r;
    }

    setAngle(a) {
        this.viewA = a;
    }

    setAlign(a) {
        this.alignCoef = a;
    }

    setSeparate(s) {
        this.separateCoef = s;
    }

    setCohesion(c) {
        this.cohesionCoef = c;
    }
    
}